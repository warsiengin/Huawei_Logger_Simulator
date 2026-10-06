import asyncio
import struct
import unittest

from server import ModbusTcpSimulator, RegisterBank, SimulatorOptions, decode_i32


def make_simulator(**overrides):
    values = {
        "listen_address": "127.0.0.1",
        "port": 15020,
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
    values.update(overrides)
    return ModbusTcpSimulator(SimulatorOptions.from_mapping(values))


class RegisterBankTests(unittest.TestCase):
    def test_guide_registers_use_big_endian_words_and_documented_gains(self):
        bank = RegisterBank(SimulatorOptions.from_mapping({}))
        self.assertEqual(bank.read(40525, 2), [0x0007, 0xA120])
        self.assertEqual(bank.read(40521, 2), [0x0007, 0xEF40])
        self.assertEqual(bank.read(40543, 1), [1])
        self.assertEqual(bank.read(40697, 2), [0, 10000])

    def test_signed_reactive_power_is_twos_complement(self):
        simulator = make_simulator(reactive_power_kvar=-12.5)
        self.assertEqual(
            decode_i32(*simulator.bank.read(40544, 2)),
            -12500,
        )

    def test_multi_register_write_updates_active_output(self):
        simulator = make_simulator()
        request = bytes((0x10,)) + struct.pack(">HHBHH", 40420, 2, 4, 0, 2500)
        self.assertEqual(
            simulator.handle_pdu(request),
            bytes((0x10,)) + struct.pack(">HH", 40420, 2),
        )
        self.assertEqual(decode_i32(*simulator.bank.read(40525, 2)), 250000)

    def test_active_power_limit_rejects_out_of_range_write(self):
        simulator = make_simulator()
        request = bytes((0x10,)) + struct.pack(">HHBHH", 40420, 2, 4, 0, 10001)
        self.assertEqual(simulator.handle_pdu(request), bytes((0x90, 3)))

    def test_cooldown_rejects_immediate_second_power_write(self):
        simulator = make_simulator()
        first = bytes((0x06,)) + struct.pack(">HH", 40421, 4000)
        second = bytes((0x06,)) + struct.pack(">HH", 40421, 3000)
        self.assertEqual(simulator.handle_pdu(first), first)
        self.assertEqual(decode_i32(*simulator.bank.read(40525, 2)), 400000)
        self.assertEqual(simulator.handle_pdu(second), bytes((0x86, 6)))

    def test_unsupported_function_returns_illegal_function(self):
        self.assertEqual(make_simulator().handle_pdu(bytes((0x04,))), bytes((0x84, 1)))

    def test_invalid_read_quantity_returns_illegal_value(self):
        request = bytes((0x03,)) + struct.pack(">HH", 40525, 126)
        self.assertEqual(make_simulator().handle_pdu(request), bytes((0x83, 3)))


class ModbusTcpTests(unittest.IsolatedAsyncioTestCase):
    async def test_mbap_transaction_reads_holding_registers(self):
        simulator = make_simulator()
        server = await asyncio.start_server(simulator.handle_client, "127.0.0.1", 0)
        address = server.sockets[0].getsockname()
        async with server:
            reader, writer = await asyncio.open_connection(*address[:2])
            pdu = bytes((0x03,)) + struct.pack(">HH", 40543, 1)
            writer.write(struct.pack(">HHHB", 7, 0, len(pdu) + 1, 0) + pdu)
            await writer.drain()
            response_header = await reader.readexactly(7)
            transaction_id, protocol_id, length, unit_id = struct.unpack(
                ">HHHB", response_header
            )
            response_pdu = await reader.readexactly(length - 1)
            writer.close()
            await writer.wait_closed()

        self.assertEqual((transaction_id, protocol_id, unit_id), (7, 0, 0))
        self.assertEqual(response_pdu, bytes((0x03, 2, 0, 1)))


if __name__ == "__main__":
    unittest.main()
