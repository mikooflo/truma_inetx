#!/usr/bin/env python3
"""Run one ventilation level then stop"""
import serial, time, sys, fcntl, termios

UART = '/dev/serial0'
ser = serial.Serial(UART, 9600, timeout=0.050)
time.sleep(0.2)
ser.reset_input_buffer()

TIOCSBRK = 0x5427
TIOCCBRK = 0x5428
RAW = {0x20: 0x20, 0x21: 0x61, 0x22: 0xE2, 0x3C: 0x3C, 0x3D: 0x7D, 0x0A: 0xCA, 0x1F: 0x1F}

def brk():
    fcntl.ioctl(ser.fd, TIOCSBRK)
    time.sleep(0.0015)
    fcntl.ioctl(ser.fd, TIOCCBRK)
    time.sleep(0.002)

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

def burst(cmd):
    ser.send_break(0.001); time.sleep(1.7); ser.reset_input_buffer()
    for si in range(3):
        wframe(0x3C, [TX_A, TX_B, TX_C][si], classic=True)
        rd = rframe(0x3D); wframe(0x20, cmd)
        td = rframe(0x21); rframe(0x22); rframe(0x0A); rframe(0x1F)
        b5 = td[5] if td else 0
        rd_hex = rd.hex()[:16] if rd else '---'
        print(f"  S2M={rd_hex} b5=0x{b5:02x}")

if len(sys.argv) < 2:
    print("Usage: test_vent_level.py <vent_level>")
    print("  vent_level: 1-10, or 'off'")
    sys.exit(1)

arg = sys.argv[1]
if arg == 'off':
    cmd = bytes.fromhex('aa aa aa 00 09 02 e0 0f')
else:
    level = int(arg)
    if level < 1 or level > 10:
        print("Level must be 1-10 or 'off'")
        sys.exit(1)
    b5 = 0x02 | (level << 4)
    cmd = bytes([0xAA, 0xAA, 0xAA, 0x00, 0x09, b5, 0xE0, 0x0F])
    cs = calc_cs(bytes([0x20]) + cmd)
    cmd = bytes([0xAA, 0xAA, 0xAA, 0x00, 0x09, b5, 0xE0, 0x0F])

print(f"Ventilation niveau {arg}...")
burst(cmd)
burst(cmd)  # second burst to confirm
print("Fait. Ctrl+C pour arrêter le script.")
print("\nAttente ... (appuie sur Ctrl+C pour quitter)")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
