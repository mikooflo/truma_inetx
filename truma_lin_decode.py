#!/usr/bin/env python3
"""
Décodeur complet du bus LIN Truma
Lit /dev/serial0 à 9600 bauds et décode toutes les trames
"""

import serial
import time
import struct
import sys

UART_PORT = '/dev/serial0'
BAUD_RATE = 9600

PID_NAMES = {
    0x20: "Command Status (CP→iNet)",
    0x21: "Status 1 - Temperatures",
    0x22: "Status 2 - AC/Power",
    0x0A: "Unknown 0A",
    0x1F: "Unknown 1F",
    0x3C: "Transport Layer Master→Slave",
    0x3D: "Transport Layer Slave→Master",
}

SID_NAMES = {
    0xB0: "Assign NAD",
    0xB1: "Assign Frame ID",
    0xB2: "Read by Identifier",
    0xB3: "Conditional Change NAD",
    0xB4: "Data Dump",
    0xB5: "Assign NAD via Position",
    0xB6: "Save Configuration",
    0xB7: "Assign Frame ID Range",
    0xB8: "Heartbeat/Status Check",
    0xB9: "Heartbeat (alt)",
    0xBA: "Data Upload (CP→iNet)",
    0xBB: "Data Download (iNet→CP)",
}

ENERGY_MIX = {0x00: "Elec", 0xFA: "Gas/Mix"}
ENERGY_MODE = {0x00: "Gas", 0x09: "Mix/Elec1", 0x12: "Mix/Elec2"}
ENERGY_MODE2 = {0x01: "Gas", 0x02: "Elec", 0x03: "Mix"}
VENT_MODE = {0x00: "Off", 0x0B: "Eco", 0x0D: "High", 0x01: "1", 0x02: "2",
             0x03: "3", 0x04: "4", 0x05: "5", 0x06: "6", 0x07: "7",
             0x08: "8", 0x09: "9", 0x0A: "10"}
CP_DISPLAY = {0xF0: "heating on", 0x20: "standby AC on", 0x00: "standby AC off",
              0xD0: "error", 0x70: "fatal error", 0x50: "boiler on", 0x40: "boiler off"}
HEAT_STATUS = {0x10: "boiler eco done", 0x11: "boiler eco heating",
               0x30: "boiler hot done", 0x31: "boiler hot heating"}
HEAT_STATUS2 = {0x04: "normal", 0x05: "error", 0xFF: "fatal error", 0xFE: "normal"}
VENT_STATUS = {0x01: "off", 0x22: "on+airvent", 0x02: "on", 0x31: "error",
               0x32: "fatal", 0x21: "airvent"}


def calc_checksum(data):
    cs = sum(data)
    while cs > 0xFF:
        cs = (cs & 0xFF) + (cs >> 8)
    cs = (~cs) & 0xFF
    return 0xFF if cs == 0 else cs


def temp_to_celsius(raw16):
    if raw16 in [0xAAA, 0xAAAA, 0x0000] or raw16 > 5000:
        return "---"
    return f"{(raw16 / 10.0 - 273):.1f}°C"


def decode_20(data):
    room_raw = data[0] | ((data[1] & 0x0F) << 8)
    water_raw = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
    emix = ENERGY_MIX.get(data[3], f"0x{data[3]:02X}")
    emode = ENERGY_MODE.get(data[4], f"0x{data[4]:02X}")
    vent = data[5] >> 4
    en2 = ENERGY_MODE2.get(data[5] & 0x0F, f"0x{data[5] & 0x0F:X}")
    vname = VENT_MODE.get(vent, f"#{vent}")
    return (f"  ├ Room: {temp_to_celsius(room_raw)}  "
            f"Water: {temp_to_celsius(water_raw)}\n"
            f"  ├ Energy: {emix} ({emode})  "
            f"El: {en2}\n"
            f"  └ Vent: {vname}  unk6={data[6]:02x} unk7={data[7]:02x}")


def decode_21(data):
    room_raw = data[0] | ((data[1] & 0x0F) << 8)
    water_raw = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
    vs = VENT_STATUS.get(data[5], f"0x{data[5]:02X}")
    return (f"  ├ Room: {temp_to_celsius(room_raw)}  "
            f"Water: {temp_to_celsius(water_raw)}\n"
            f"  └ b3={data[3]:02x} b4={data[4]:02x} "
            f"state={vs} b6={data[6]:02x} b7={data[7]:02x}")


def decode_22(data):
    v = data[0] / 10.0
    cp = CP_DISPLAY.get(data[1], f"0x{data[1]:02X}")
    hs = HEAT_STATUS.get(data[2], f"0x{data[2]:02X}")
    hs2 = HEAT_STATUS2.get(data[3], f"0x{data[3]:02X}")
    return (f"  ├ Voltage: {v}V  CP+: {cp}\n"
            f"  └ Heat: {hs} ({hs2})  "
            f"b4={data[4]:02x} b5={data[5]:02x} b6={data[6]:02x} b7={data[7]:02x}")


def decode_transport_m2s(payload):
    nad = payload[0]
    pci = payload[1]
    pci_type = pci >> 4
    nad_str = "BROADCAST" if nad == 0x7F else f"0x{nad:02x}"
    if pci_type == 0:  # single frame
        expected = (pci & 0x0F) - 1
        sid = payload[2]
        pl = payload[3:3 + expected]
        sname = SID_NAMES.get(sid, f"0x{sid:02X}")
        return f"  ├ NAD={nad_str} Single SID={sname} ({len(pl)}b)\n  └ Data: {' '.join(f'{b:02x}' for b in pl)}"
    elif pci_type == 1:  # first frame
        expected = ((pci & 0x0F) << 8) | payload[2]
        sid = payload[3]
        pl = payload[4:]
        sname = SID_NAMES.get(sid, f"0x{sid:02X}")
        return f"  ├ NAD={nad_str} First SID={sname} ({expected}b)\n  └ Data: {' '.join(f'{b:02x}' for b in pl)}"
    elif pci_type == 2:
        return f"  └ NAD={nad_str} Consecutive ({pci & 0x0F})"
    return f"  └ NAD={nad_str} Unknown frame type {pci_type}"


def decode_transport_s2m(payload):
    nad = payload[0]
    pci = payload[1]
    pci_type = pci >> 4
    if pci_type == 0:
        expected = (pci & 0x0F) - 1
        rsid = payload[2]
        pl = payload[3:3 + expected]
        if rsid == 0x7F:
            return f"  └ NAD=0x{nad:02x} NEGATIVE RESP err={payload[3]:02x}"
        sid_resp = rsid - 0x40
        sname = SID_NAMES.get(sid_resp, f"0x{sid_resp:02X}")
        return f"  └ NAD=0x{nad:02x} Response to {sname}: {' '.join(f'{b:02x}' for b in pl)}"
    elif pci_type == 1:
        expected = ((pci & 0x0F) << 8) | payload[2]
        rsid = payload[3]
        pl = payload[4:]
        return f"  └ NAD=0x{nad:02x} First frame ({expected}b) RSID=0x{rsid:02x}: {' '.join(f'{b:02x}' for b in pl)}"
    elif pci_type == 2:
        return f"  └ NAD=0x{nad:02x} Consecutive ({pci & 0x0F}): {' '.join(f'{b:02x}' for b in payload[2:])}"
    return f"  └ NAD=0x{nad:02x} Unknown"


def decode_frame(msg):
    sync0, sync1, pid_raw = msg[0], msg[1], msg[2]
    payload = msg[3:11]
    cs = msg[11]
    pid = pid_raw & 0x3F

    # Checksum: classic (data only) for transport, enhanced (PID+data) for others
    if pid in [0x3C, 0x3D]:
        cs_ok = calc_checksum(payload) == cs
    else:
        cs_ok = calc_checksum(bytes([pid_raw]) + payload) == cs

    lines = []
    pname = PID_NAMES.get(pid, f"0x{pid:02X}")
    lines.append(f"┌─ PID 0x{pid:02X} ({pname})  raw=0x{pid_raw:02X}  "
                 f"cs={'✓' if cs_ok else '✗'}")

    if pid == 0x20:
        lines.append(decode_20(payload))
    elif pid == 0x21:
        lines.append(decode_21(payload))
    elif pid == 0x22:
        lines.append(decode_22(payload))
    elif pid == 0x3C:
        lines.append(decode_transport_m2s(payload))
    elif pid == 0x3D:
        lines.append(decode_transport_s2m(payload))
    elif pid == 0x0A:
        if payload == bytes(8):
            lines.append("  └ (all zeros)")
        else:
            lines.append(f"  └ {' '.join(f'{b:02x}' for b in payload)}")
    elif pid == 0x1F:
        lines.append(f"  └ {' '.join(f'{b:02x}' for b in payload)}")
    else:
        lines.append(f"  └ RAW: {' '.join(f'{b:02x}' for b in payload)}")

    return '\n'.join(lines)


def main():
    print(f"Connexion à {UART_PORT} à {BAUD_RATE} bauds...\n")
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()

    buffer = bytearray()
    msg_count = 0
    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                while len(buffer) >= 3:
                    synced = -1
                    for i in range(len(buffer) - 1):
                        if buffer[i] == 0x00 and buffer[i + 1] == 0x55:
                            synced = i
                            break
                    if synced == -1:
                        buffer.clear()
                        break
                    if synced > 0:
                        buffer = buffer[synced:]
                    if len(buffer) >= 12:
                        msg = buffer[:12]
                        buffer = buffer[12:]
                        msg_count += 1
                        print(f"\n--- Trame #{msg_count} ---")
                        print(decode_frame(msg))
                    else:
                        break
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nArrêt.")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
