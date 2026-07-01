# Truma iNetX LIN Master Replacement

Replace the Truma iNetX board with a Raspberry Pi as the LIN bus master to read sensor data, send heating/ventilation commands, and publish everything over MQTT.

## Hardware Requirements

### LIN Transceiver
- **TJA1020** or **TJA1021** LIN transceiver (or similar)
- Typical pinout:
  - Pin 1 (TXD) → Raspberry Pi TX (GPIO14)
  - Pin 2 (RXD) → Raspberry Pi RX (GPIO15)
  - Pin 3 (NSLP) → Raspberry Pi GPIO27 (or GND for always-on)
  - Pin 4 (NWAKE) → +5V (inactive)
  - Pin 5 (GND) → GND
  - Pin 6 (LIN) → LIN bus (Truma pin 9)
  - Pin 7 (VBAT) → +12V (fused)
  - Pin 8 (TXD) → N/C on TJA1021

### Pull-Up Resistor & Diode
- **1 kΩ resistor** + **1N4148 diode** in series between VBAT (+12V) and the LIN bus line.
- This provides the required LIN pull-up to battery voltage.
- Without it, the LIN bus voltage may be too low for reliable communication.

### UART
- Uses `/dev/serial0` → `ttyS0` (mini UART on Raspberry Pi).
- **BAUD: 9600** (LIN standard).
- The mini UART has no hardware break generation — breaks are done via `ioctl(TIOCSBRK/TIOCCBRK)` for 1.5 ms.

### Wiring Summary
```
Raspberry Pi              TJA1020/21
GPIO14 (TXD)  ──────────  TXD (pin 1)
GPIO15 (RXD)  ──────────  RXD (pin 2)
GPIO27        ──────────  NSLP (pin 3) — or GND for always-on
GND           ──────────  GND (pin 5)
                          LIN (pin 6) ──┬── 1kΩ + 1N4148 ── +12V
                                        │
                                        └── Truma pin 9 (LIN bus)
                          VBAT (pin 7) ── +12V (fused)
```

## Software Dependencies

- Python 3.9+
- `pyserial` (UART communication)
- `paho-mqtt` (MQTT client)
- Mosquitto (MQTT broker, localhost:1883)

All dependencies are included in the virtualenv at `/root/venv/bin/python3`.

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/mikooflo/truma_inetx.git
   cd truma_inetx
   ```

2. Install the systemd service:
   ```
   cp truma_master.service /etc/systemd/system/
   systemctl daemon-reload
   systemctl enable truma_master.service
   systemctl start truma_master.service
   ```

3. Logs are written to `/var/log/mqtt_scripts/truma_master.log`.

## How It Works

### LIN Frame Structure

Each sub-cycle sends 7 frames:

| PID | Direction | Content |
|-----|-----------|---------|
| 0x3C | Master → Slave | Truma heartbeat (read-by-ID / header) |
| 0x3D | Slave → Master | Truma response (heartbeat status) |
| 0x20 | Master → Slave | Command/status (setpoint temperatures, modes, power) |
| 0x21 | Slave → Master | Temperature data (room + water) |
| 0x22 | Slave → Master | AC mains detection |
| 0x0A | Slave → Master | Unknown (read for bus sync) |
| 0x1F | Slave → Master | Unknown (read for bus sync) |

### Sub-Cycle Types

Three sub-cycles run sequentially in every burst:

- **A**: Read-by-ID 0xB2, type 0x00
- **B**: Heartbeat 0xB8
- **C**: Read-by-ID 0xB2, type 0x23

### Idle vs Active Mode

| State | Sub-cycles | Cycle Time | When |
|-------|-----------|------------|------|
| **Idle** | 3 (A + B + C) | ~12 seconds | No command active |
| **Active** | 2 (A + B) | ~700 ms | Any heating/ventilation command active |

In idle mode, a 1 ms LIN break + 1.7 s idle gap is sent before each burst to keep the Truma awake without flooding the bus.

### Temperature Decoding

- **Room temperature**: 12-bit raw value in bytes 0-1. Valid range 2630–3330 (≈ -10°C to +60°C). Formula: `raw / 10.0 - 273`.
- **Water temperature**: 12-bit raw value in bytes 1-2. Valid range 2730–3730 (≈ 0°C to +100°C). Formula: `raw / 10.0 - 273`.
- Values outside valid ranges are discarded (protects against -273°C spikes).
- A rolling median of the last 3 readings is published to filter out transient errors.

### Command Frame (PID 0x20) Encoding

| Byte | Bits | Field |
|------|------|-------|
| 0 | 8 | Room setpoint LSB |
| 1 | 4+4 | Room setpoint MSB + Water setpoint MSB |
| 2 | 8 | Water setpoint LSB |
| 3 | 8 | Energy mix (0x00 = elec, 0xFA = gas) |
| 4 | 8 | Power (0x09 = 900W, 0x12 = 1800W) |
| 5 | 4+4 | Ventilation mode + Energy source (01=gas, 02=elec, 03=mix) |
| 6-7 | — | Always 0xE0, 0x0F |

Water mode raw values:
- `0xAAA` = off
- `0xC30` = eco (~40°C target)
- `0xCD0` = comfort (~55°C target)
- `0xD00` = hot (~60°C target)

### Water Heating Hysteresis

To prevent short-cycling due to thermal inertia (~3-5°C overshoot with full tank), water heating is cut off at `target - 4°C`:
- Eco: cutoff at 35°C
- Comfort: cutoff at 51°C
- Hot: cutoff at 56°C

When the cutoff is reached, `0xAAA` (off) is sent in the command frame instead of the setpoint value. The `water_heating` MQTT topic reflects this logic.

### 230V Mains Detection (AC Input)

Detected from the AC status frame (PID 0x22), byte 1, bit 5 (0x20):
- `0x70` = 230V present
- `0x50` = 230V absent

Published as `acin` topic (`ON`/`OFF`).

### Wake-Up Sequence

On startup, the Truma may be asleep. A dummy break + header (0xFF) + 1 second wait is sent to wake it. The first frame after wake-up always fails — it is discarded.

### Dual-Master Handover

The iNetX module can remain physically connected (powered off). If it powers on:

1. **Bus activity detection**: At startup (2 seconds) and during operation (every 50 ms), the Pi checks for incoming data.
2. **Listener mode**: When iNetX activity is detected, the Pi stops transmitting and passively listens/decodes frames. It publishes `master=inetx` and mirrors decoded status to MQTT.
3. **Reclaiming master**: After 30 seconds of bus silence (iNetX powered off), the Pi resumes master mode and publishes `master=pi`.

This allows seamless failover without bus contention.

### MQTT Topics

Base: `service/truma/`

#### Published (status)

| Topic | Values | Retained |
|-------|--------|----------|
| `room_temp` | Float (°C) | Yes |
| `water_temp` | Float (°C) | Yes |
| `air_mode` | `off`, `eco`, `high`, `vent` | Yes |
| `water_mode` | `off`, `eco`, `comfort`, `hot` | Yes |
| `water_heating` | `ON`, `OFF` | Yes |
| `ventilation` | 0–10 | Yes |
| `energy` | `elec`, `gas`, `mix` | Yes |
| `power` | 900, 1800 | Yes |
| `acin` | `ON`, `OFF` | Yes |
| `master` | `pi`, `inetx` | Yes |

#### Subscribed (commands)

| Topic | Values |
|-------|--------|
| `cmd/air_mode` | `off`, `eco`, `high`, `vent` |
| `cmd/room_temp` | Float (°C) or `off` |
| `cmd/water_mode` | `off`, `eco`, `comfort`, `hot` |
| `cmd/ventilation` | 0–10 |
| `cmd/energy` | `elec`, `gas`, `mix` |
| `cmd/power` | 900, 1800 |

#### Startup Safety

On service start, all command topics are published with `off`/`0` (retained) before subscribing. This ensures any previously retained command cannot accidentally activate heating on reboot.

### Checksum

- **Classic** (PID 0x3C, 0x3D): Sum of data bytes only.
- **Enhanced** (PID 0x20): Sum of PID + data bytes.

## License

Internal project — no license specified.
