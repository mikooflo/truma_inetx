#!/usr/bin/env python3
"""Test water heating"""
import serial, time, sys, fcntl, termios

UART = '/dev/serial0'
ser = serial.Serial(UART, 9600, timeout=0.050)
time.sleep(0.2)
ser.reset_input_buffer()

TIOCSBRK = 0x5427
TIOCCBRK = 0x5428

def brk():
    fcntl.ioctl(ser.fd, TIOCSBRK)
    time.sleep(0.0015)
    fcntl.ioctl(ser.fd, TIOCCBRK)
    time.sleep(0.002)

RAW = {0x20: 0x20, 0x21: 0x61, 0x22: 0xE2, 0x3C: 0x3C, 0x3D: 0x7D, 0x0A: 0xCA, 0x1F: 0x1F}

def calc_cs(data):
    cs = sum(data)
    while cs > 0xFF: cs = (cs & 0xFF) + (cs >> 8)
    return (~cs) & 0xFF or 0xFF

def wframe(pid, data=None, classic=False):
    ser.reset_input_buffer(); brk()
    raw = RAW[pid]; ser.write(bytes([0x55, raw])); ser.flush()
    if data is not None:
        cs = calc_cs(data if classic else bytes([raw]) + data)
        ser.write(data + bytes([cs])); ser.flush()
        ser.reset_input_buffer()

def rframe(pid):
    ser.reset_input_buffer(); brk()
    raw = RAW[pid]; ser.write(bytes([0x55, raw])); ser.flush()
    resp = ser.read(100)
    for i in range(len(resp)-5):
        if resp[i]==0x55 and resp[i+1]==raw: return resp[i+2:i+10]
    return resp[-9:-1] if len(resp)>=12 else None

TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
TX = [TX_A, TX_B, TX_C]

# --- 1. Water 55°C eco elec ---
water_raw = 0xCD0
room_raw = 0xAAA
b0 = room_raw & 0xFF
b1 = ((room_raw >> 8) & 0x0F) | ((water_raw & 0x0F) << 4)
b2 = (water_raw >> 4) & 0xFF
cmd = bytes([b0, b1, b2, 0x00, 0x09, 0xB2, 0xE0, 0x0F])

def burst(cmd, label):
    print(f"\n=== {label} ===")
    ser.send_break(0.001); time.sleep(1.7); ser.reset_input_buffer()
    for si in range(3):
        wframe(0x3C, TX[si], classic=True)
        rd = rframe(0x3D); wframe(0x20, cmd)
        td = rframe(0x21); rframe(0x22); rframe(0x0A); rframe(0x1F)
        if td:
            r = td[0] | ((td[1] & 0x0F) << 8)
            w = (td[2] << 4) | ((td[1] & 0xF0) >> 4)
            room = r/10-273 if r < 5000 else 0
            water = w/10-273 if w < 5000 else 0
            b5 = td[5]
            rd_hex = rd.hex()[:16] if rd else '---'
            print(f"  Room={room:.1f}°C Water={water:.1f}°C b5=0x{b5:02x} S2M={rd_hex}")

burst(cmd, "Eau 55°C eco")
if len(sys.argv) > 1:
    burst(cmd, "Eau 55°C eco (confirm)")

print("\nFait. (Ctrl+C pour quitter)")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
