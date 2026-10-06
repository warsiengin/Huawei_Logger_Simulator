#!/usr/bin/env python3
"""Small Modbus-TCP simulator for the SmartLogger SCADA guide profile."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import signal
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

LOGGER = logging.getLogger("smartlogger_simulator")
OPTIONS_PATH = Path(os.environ.get("OPTIONS_FILE", "/data/options.json"))
MAX_REGISTER_ADDRESS = 50001
MAX_ENERGY_KWH = 0xFFFFFFFF / 10


class ModbusException(Exception):
    def __init__(self, code: int) -> None:
        super().__init__(f"Modbus exception {code}")
        self.code = code


@dataclass(frozen=True)
class SimulatorOptions:
    listen_address: str
    port: int
    unit_id: int
    request_timeout_seconds: int
    write_cooldown_seconds: int
    active_power_kw: float
    reactive_power_kvar: float
    input_power_kw: float
    power_factor: float
    daily_energy_kwh: float
    total_energy_kwh: float
    plant_status: int
    alarm_info_1: int
    alarm_info_2: int
    active_power_limit_kw: float

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> SimulatorOptions:
        defaults: dict[str, Any] = {
            "listen_address": "0.0.0.0",
            "port": 502,
            "unit_id": 0,
            "request_timeout_seconds": 5,
            "write_cooldown_seconds": 1,
            "active_power_kw": 500,
            "reactive_power_kvar": 0,
            "input_power_kw": 520,
            "power_factor": 0.99,
            "daily_energy_kwh": 1250,
            "total_energy_kwh": 250000,
            "plant_status": 1,
            "alarm_info_1": 0,
            "alarm_info_2": 0,
            "active_power_limit_kw": 1000,
        }
        values = {**defaults, **raw}
        try:
            options = cls(
                listen_address=str(values["listen_address"]),
                port=int(values["port"]),
                unit_id=int(values["unit_id"]),
                request_timeout_seconds=int(values["request_timeout_seconds"]),
                write_cooldown_seconds=int(values["write_cooldown_seconds"]),
                active_power_kw=float(values["active_power_kw"]),
                reactive_power_kvar=float(values["reactive_power_kvar"]),
                input_power_kw=float(values["input_power_kw"]),
                power_factor=float(values["power_factor"]),
                daily_energy_kwh=float(values["daily_energy_kwh"]),
                total_energy_kwh=float(values["total_energy_kwh"]),
                plant_status=int(values["plant_status"]),
                alarm_info_1=int(values["alarm_info_1"]),
                alarm_info_2=int(values["alarm_info_2"]),
                active_power_limit_kw=float(values["active_power_limit_kw"]),
            )
        except (TypeError, ValueError) as err:
            raise ValueError(f"Invalid add-on option: {err}") from err

        if not options.listen_address:
            raise ValueError("listen_address must not be empty")
        if not 1 <= options.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not 0 <= options.unit_id <= 255:
            raise ValueError("unit_id must be between 0 and 255")
        if not 1 <= options.request_timeout_seconds <= 60:
            raise ValueError("request_timeout_seconds must be between 1 and 60")
        if not 1 <= options.write_cooldown_seconds <= 60:
            raise ValueError("write_cooldown_seconds must be between 1 and 60")
        finite_values = (
            options.active_power_kw,
            options.reactive_power_kvar,
            options.input_power_kw,
            options.power_factor,
            options.daily_energy_kwh,
            options.total_energy_kwh,
            options.active_power_limit_kw,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("Numeric simulation options must be finite")
        if options.active_power_kw < 0 or options.input_power_kw < 0:
            raise ValueError("active_power_kw and input_power_kw must be non-negative")
        if not -1 <= options.power_factor <= 1:
            raise ValueError("power_factor must be between -1 and 1")
        if options.daily_energy_kwh < 0 or options.total_energy_kwh < 0:
            raise ValueError("Energy values must be non-negative")
        if not 1 <= options.plant_status <= 5:
            raise ValueError("plant_status must be between 1 and 5")
        if not 0 <= options.alarm_info_1 <= 65535 or not 0 <= options.alarm_info_2 <= 65535:
            raise ValueError("Alarm values must be between 0 and 65535")
        if options.active_power_limit_kw < 0:
            raise ValueError("active_power_limit_kw must be non-negative")
        if options.active_power_kw > options.active_power_limit_kw:
            raise ValueError("active_power_kw must not exceed active_power_limit_kw")

        return options


def encode_u32(value: int) -> tuple[int, int]:
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("Unsigned 32-bit value is out of range")
    return (value >> 16, value & 0xFFFF)


def encode_i32(value: int) -> tuple[int, int]:
    if not -0x80000000 <= value <= 0x7FFFFFFF:
        raise ValueError("Signed 32-bit value is out of range")
    return encode_u32(value & 0xFFFFFFFF)


def decode_i32(high: int, low: int) -> int:
    value = (high << 16) | low
    return value - 0x100000000 if value & 0x80000000 else value


class RegisterBank:
    """Guide-addressed holding registers; multiword values use MSW-first order."""

    POWER_WORDS = {
        40420: ("active", "unsigned"),
        40421: ("active", "unsigned"),
        40422: ("reactive", "signed"),
        40423: ("reactive", "signed"),
        40424: ("failsafe", "unsigned"),
        40425: ("failsafe", "unsigned"),
    }

    def __init__(self, options: SimulatorOptions) -> None:
        self.options = options
        self.registers: dict[int, int] = {}
        self.last_power_write = float("-inf")
        self._last_energy_update = time.monotonic()
        self.daily_energy_kwh = options.daily_energy_kwh
        self.total_energy_kwh = options.total_energy_kwh
        self.active_power_kw = options.active_power_kw
        self.reactive_power_kvar = options.reactive_power_kvar

        active_raw = self._scale(options.active_power_kw, 10)
        reactive_raw = self._scale(options.reactive_power_kvar, 10, signed=True)
        self._set_pair(40420, active_raw)
        self._set_pair(40422, reactive_raw, signed=True)
        self._set_pair(40424, active_raw)
        self._update_telemetry()

    @staticmethod
    def _scale(value: float, gain: int, signed: bool = False) -> int:
        raw = round(value * gain)
        minimum, maximum = (-0x80000000, 0x7FFFFFFF) if signed else (0, 0xFFFFFFFF)
        if not minimum <= raw <= maximum:
            raise ValueError(f"Scaled register value is out of range: {value}")
        return raw

    def _set_pair(self, address: int, value: int, signed: bool = False) -> None:
        words = encode_i32(value) if signed else encode_u32(value)
        self.registers[address], self.registers[address + 1] = words

    def _update_energy(self) -> None:
        now = time.monotonic()
        elapsed_hours = max(0.0, now - self._last_energy_update) / 3600
        generated_kwh = self.active_power_kw * elapsed_hours
        self.daily_energy_kwh = min(MAX_ENERGY_KWH, self.daily_energy_kwh + generated_kwh)
        self.total_energy_kwh = min(MAX_ENERGY_KWH, self.total_energy_kwh + generated_kwh)
        self._last_energy_update = now

    def _update_telemetry(self) -> None:
        active_raw = self._scale(self.active_power_kw, 1000, signed=True)
        reactive_raw = self._scale(self.reactive_power_kvar, 1000, signed=True)
        input_raw = self._scale(self.options.input_power_kw, 1000)
        daily_raw = self._scale(self.daily_energy_kwh, 10)
        total_raw = self._scale(self.total_energy_kwh, 10)
        self._set_pair(40525, active_raw, signed=True)
        self._set_pair(40544, reactive_raw, signed=True)
        self._set_pair(40521, input_raw)
        self.registers[40532] = round(self.options.power_factor * 1000) & 0xFFFF
        self._set_pair(40562, daily_raw)
        self._set_pair(40560, total_raw)
        self.registers[40543] = self.options.plant_status
        self.registers[50000] = self.options.alarm_info_1
        self.registers[50001] = self.options.alarm_info_2
        self._set_pair(40697, self._scale(self.options.active_power_limit_kw, 10))

    def read(self, address: int, quantity: int) -> list[int]:
        if quantity < 1 or quantity > 125:
            raise ModbusException(3)
        self._check_range(address, quantity)
        self._update_energy()
        self._update_telemetry()
        return [self.registers.get(index, 0) for index in range(address, address + quantity)]

    def write(self, address: int, values: list[int]) -> None:
        if not values or len(values) > 123 or any(not 0 <= value <= 0xFFFF for value in values):
            raise ModbusException(3)
        self._check_range(address, len(values))
        targets = range(address, address + len(values))
        if any(index not in self.POWER_WORDS for index in targets):
            raise ModbusException(2)

        now = time.monotonic()
        if now - self.last_power_write < self.options.write_cooldown_seconds:
            raise ModbusException(6)

        candidate = dict(self.registers)
        candidate.update(zip(targets, values))
        changed_power_groups = {self.POWER_WORDS[index][0] for index in targets}
        for group in changed_power_groups:
            first_address = next(
                index for index, (name, _) in self.POWER_WORDS.items() if name == group
            )
            high = candidate[first_address]
            low = candidate[first_address + 1]
            kind = self.POWER_WORDS[first_address][1]
            target = decode_i32(high, low) if kind == "signed" else (high << 16) | low
            if group in ("active", "failsafe"):
                limit = self._scale(self.options.active_power_limit_kw, 10)
                if target > limit:
                    raise ModbusException(3)

        self.registers.update(zip(targets, values))
        self._apply_setpoints(changed_power_groups)
        self.last_power_write = now

    def _apply_setpoints(self, groups: set[str]) -> None:
        if "active" in groups or "failsafe" in groups:
            group = "active" if "active" in groups else "failsafe"
            address = 40420 if group == "active" else 40424
            setpoint_kw = ((self.registers[address] << 16) | self.registers[address + 1]) / 10
            self.active_power_kw = min(self.options.active_power_kw, setpoint_kw)
        if "reactive" in groups:
            self.reactive_power_kvar = decode_i32(self.registers[40422], self.registers[40423]) / 10
        self._update_telemetry()

    @staticmethod
    def _check_range(address: int, quantity: int) -> None:
        if address < 0 or address + quantity - 1 > MAX_REGISTER_ADDRESS:
            raise ModbusException(2)


class ModbusTcpSimulator:
    def __init__(self, options: SimulatorOptions) -> None:
        self.options = options
        self.bank = RegisterBank(options)

    def handle_pdu(self, pdu: bytes) -> bytes:
        if not pdu:
            raise ModbusException(3)
        function = pdu[0]
        try:
            if function == 0x03:
                if len(pdu) != 5:
                    raise ModbusException(3)
                address, quantity = struct.unpack(">HH", pdu[1:])
                values = self.bank.read(address, quantity)
                return bytes((function, len(values) * 2)) + b"".join(
                    struct.pack(">H", value) for value in values
                )
            if function == 0x06:
                if len(pdu) != 5:
                    raise ModbusException(3)
                address, value = struct.unpack(">HH", pdu[1:])
                self.bank.write(address, [value])
                return pdu
            if function == 0x10:
                if len(pdu) < 6:
                    raise ModbusException(3)
                address, quantity, byte_count = struct.unpack(">HHB", pdu[1:6])
                if quantity < 1 or quantity > 123 or byte_count != quantity * 2:
                    raise ModbusException(3)
                if len(pdu) != 6 + byte_count:
                    raise ModbusException(3)
                values = list(struct.unpack(f">{quantity}H", pdu[6:]))
                self.bank.write(address, values)
                return bytes((function,)) + struct.pack(">HH", address, quantity)
            raise ModbusException(1)
        except ModbusException as err:
            return bytes((function | 0x80, err.code))

    async def serve(self) -> None:
        server = await asyncio.start_server(
            self.handle_client,
            self.options.listen_address,
            self.options.port,
        )
        addresses = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
        LOGGER.info(
            "SmartLogger Modbus-TCP simulator listening on %s (unit ID %d)",
            addresses,
            self.options.unit_id,
        )
        async with server:
            await server.serve_forever()

    async def handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        LOGGER.info("SCADA client connected: %s", peer)
        loop = asyncio.get_running_loop()
        try:
            while True:
                deadline = loop.time() + self.options.request_timeout_seconds
                try:
                    header = await asyncio.wait_for(
                        reader.readexactly(7), timeout=max(0, deadline - loop.time())
                    )
                except (asyncio.IncompleteReadError, asyncio.TimeoutError):
                    break
                transaction_id, protocol_id, length, unit_id = struct.unpack(">HHHB", header)
                if protocol_id != 0 or not 2 <= length <= 254:
                    LOGGER.warning("Closing malformed Modbus frame from %s", peer)
                    break
                try:
                    pdu = await asyncio.wait_for(
                        reader.readexactly(length - 1),
                        timeout=max(0, deadline - loop.time()),
                    )
                except (asyncio.IncompleteReadError, asyncio.TimeoutError):
                    break
                if unit_id != self.options.unit_id:
                    LOGGER.debug("Ignoring request for unmatched unit ID %d", unit_id)
                    continue
                response_pdu = self.handle_pdu(pdu)
                response = struct.pack(
                    ">HHHB", transaction_id, 0, len(response_pdu) + 1, unit_id
                ) + response_pdu
                writer.write(response)
                await writer.drain()
        except (ConnectionError, OSError) as err:
            LOGGER.warning("Modbus client connection error from %s: %s", peer, err)
        finally:
            writer.close()
            await writer.wait_closed()
            LOGGER.info("SCADA client disconnected: %s", peer)


def load_options() -> SimulatorOptions:
    if not OPTIONS_PATH.exists():
        LOGGER.warning("Options file %s is missing; using simulator defaults", OPTIONS_PATH)
        raw: dict[str, Any] = {}
    else:
        with OPTIONS_PATH.open(encoding="utf-8") as options_file:
            raw = json.load(options_file)
        if not isinstance(raw, dict):
            raise ValueError("Add-on options must be a JSON object")
    return SimulatorOptions.from_mapping(raw)


async def main() -> None:
    logging.basicConfig(
        level=os.environ.get("LOG_LEVEL", "INFO").upper(),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    simulator = ModbusTcpSimulator(load_options())
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    for stop_signal in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(stop_signal, stop.set)
    server_task = asyncio.create_task(simulator.serve())
    stop_task = asyncio.create_task(stop.wait())
    done, pending = await asyncio.wait(
        (server_task, stop_task), return_when=asyncio.FIRST_COMPLETED
    )
    for task in pending:
        task.cancel()
    if pending:
        await asyncio.gather(*pending, return_exceptions=True)
    for task in done:
        task.result()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (OSError, ValueError, json.JSONDecodeError) as err:
        LOGGER.error("Unable to start simulator: %s", err)
        raise SystemExit(1) from err
