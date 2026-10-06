# SmartLogger Modbus Simulator add-on

This Home Assistant add-on simulates the **SmartLogger V300R024C10SPC161**
Modbus-TCP SCADA profile in `docs/SmartLogger_Modbus_SCADA_Guide.pdf`. It provides
holding-register telemetry, writable power adjustment registers, and the
guide's communication constraints without requiring a physical logger.

## Install

1. Add this repository as a local add-on repository in Home Assistant, or copy
   this repository into the Home Assistant add-ons directory.
2. Install **SmartLogger Modbus Simulator** from the add-on store.
3. Set its options and start it.
4. Point a Modbus-TCP client at the IP address of the Home Assistant host and
   the configured port (default `502`).

The add-on uses host networking so SCADA clients can connect directly to the
Home Assistant host. `listen_address` selects the local interface to bind to;
use `0.0.0.0` to listen on all interfaces. The default unit ID is the guide's
fixed target ID `0`. Port `502` may require elevated privileges on the host.
Modbus-TCP has no authentication or encryption; expose the port only on a
trusted, appropriately firewalled network.

## Add-on options

| Option | Default | Description |
| --- | ---: | --- |
| `listen_address` | `0.0.0.0` | Local IP/interface on which to listen |
| `port` | `502` | Modbus-TCP port |
| `unit_id` | `0` | Modbus unit/slave ID; the guide profile uses `0` |
| `request_timeout_seconds` | `5` | Maximum time allowed for a complete Modbus request |
| `write_cooldown_seconds` | `1` | Minimum delay between power-register writes |
| `active_power_kw` | `500` | Simulated active output and initial active setpoint |
| `reactive_power_kvar` | `0` | Simulated reactive output and initial reactive setpoint |
| `input_power_kw` | `520` | Simulated DC input power |
| `power_factor` | `0.99` | Simulated power factor, from `-1` to `1` |
| `daily_energy_kwh` | `1250` | Initial daily energy; grows with simulated output |
| `total_energy_kwh` | `250000` | Initial lifetime energy; grows with simulated output |
| `plant_status` | `1` | Status code: 1 unlimited, 2 limited, 3 idle, 4 outage, 5 communication interrupt |
| `alarm_info_1` | `0` | Initial mask for register `50000` (bit 3 active schedule; bit 11 reactive schedule) |
| `alarm_info_2` | `0` | Initial mask for register `50001` (bit 1 MCB disconnect; bit 2 abnormal cubicle; bit 3 address conflict) |
| `active_power_limit_kw` | `1000` | Write guardrail and value exposed at register `40697` |

Configuration changes take effect when the add-on restarts. Power-control
writes update simulated output; energy registers accumulate using that output.
Alarm values are configurable bitmasks, not decoded alarm descriptions.

## Supported Modbus operations

| Function | Operation |
| --- | --- |
| `0x03` | Read Holding Registers (1–125 registers) |
| `0x06` | Write Single Register |
| `0x10` | Write Multiple Registers (1–123 registers) |

The server rejects unsupported function codes, invalid quantities, writes to
read-only/unmapped registers, and active power setpoints above register
`40697`. Writes to the active and reactive power adjustment blocks observe the
configured cooldown. A request addressed to a different unit ID is ignored.
The TCP transaction is bounded by the configured request timeout (5 seconds by
default).

## Register map

Register addresses below use the exact address numbers printed in the guide
as the Modbus PDU starting address. Multi-register values are big-endian
(most-significant register first), as in the guide. Engineering values are
encoded as `raw = value × gain`; divide raw values by the gain when decoding.

| Parameter | Address | Qty | Type | Gain | Unit | Access |
| --- | ---: | ---: | --- | ---: | --- | --- |
| Active adjustment (volatile) | `40420` | 2 | U32 | 10 | kW | Read/write |
| Reactive adjustment | `40422` | 2 | I32 | 10 | kvar | Read/write |
| Active adjustment (fail-safe) | `40424` | 2 | U32 | 10 | kW | Read/write |
| Input power | `40521` | 2 | U32 | 1000 | kW | Read-only |
| Active power | `40525` | 2 | I32 | 1000 | kW | Read-only |
| Power factor | `40532` | 1 | I16 | 1000 | — | Read-only |
| Plant status | `40543` | 1 | U16 | 1 | — | Read-only |
| Reactive power | `40544` | 2 | I32 | 1000 | kvar | Read-only |
| Total energy | `40560` | 2 | U32 | 10 | kWh | Read-only |
| Daily energy | `40562` | 2 | U32 | 10 | kWh | Read-only |
| Active power limit | `40697` | 2 | U32 | 10 | kW | Read-only |
| Alarm info 1 | `50000` | 1 | U16 | 1 | — | Read-only |
| Alarm info 2 | `50001` | 1 | U16 | 1 | — | Read-only |

For U32/I32 writes, send both words together with function `0x10` to avoid
half-word updates. `0x06` is supported for isolated single-register writes as
specified by the guide.

## Development

The server uses only the Python standard library. Run the protocol tests from
the repository root:

```sh
python -m unittest discover -s tests -v
```

Run a local instance by setting the `OPTIONS_FILE` environment variable to a
JSON file containing the add-on options. In Home Assistant, options are read
from `/data/options.json`.
