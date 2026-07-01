#!/usr/bin/env python3
"""
Test: rafale style iNetX (trames serrées), break ioctl 13ms,
puis pause 13s comme l'iNetX.
"""
import serial, time, sys, fcntl, termios

UART = '/dev/serial0'
ser = serial.Serial(UART, 9600, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

TIOCSBRK = 0x5427
TIOCCBRK = 0x5428

def brk(ms=13):
    fcntl.ioctl(ser.fd, TIOCSBRK)
    time.sleep(ms/1000.0)
    fcntl.ioctl(ser.fd, TIOCCBRK)
    time.sleep(0.002)

RAW = {0x20: 0x20, 0x21: 0x61, 0x22: 0xE2, 0x3C: 0x3C, 0x3D: 0x7D, 0x0A: 0xCA, 0x1F: 0x1F}

def calc_cs(data):
    cs = sum(data)
    while cs > 0xFF: cs = (cs & 0xFF) + (cs >> 8)
    return (~cs) & 0xFF or 0xFF

def write_frame(pid, data=None, classic=False):
    ser.reset_input_buffer()
    brk(13)
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()
    if data is not None:
        cs = calc_cs(data if classic else bytes([raw]) + data)
        time.sleep(0.002)
        ser.write(data + bytes([cs]))
        ser.flush()
        time.sleep(0.020)
        ser.reset_input_buffer()

def read_frame(pid):
    ser.reset_input_buffer()
    brk(13)
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()
    time.sleep(0.050)
    resp = ser.read(100)
    for i in range(len(resp)-5):
        if resp[i] == 0x55 and resp[i+1] == raw:
            d = resp[i+2:i+10]
            return d
    if len(resp) >= 12:
        return resp[-9:-1]
    return None

# Wake-up
ser.send_break(0.020)
time.sleep(0.005)
ser.write(bytes([0x55, 0xFF]))
time.sleep(1.0)
ser.reset_input_buffer()

cmdstat = bytes.fromhex('c2 ab aa 00 09 b2 e0 0f')
TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
TX = [TX_A, TX_B, TX_C]
NAMES = ['ReadA', 'Heart', 'ReadB']

def burst(sub_idx):
    write_frame(0x3C, TX[sub_idx], classic=True)
    read_frame(0x3D)
    write_frame(0x20, cmdstat)
    t = read_frame(0x21)
    a = read_frame(0x22)
    read_frame(0x0A)
    read_frame(0x1F)
    if t:
        r = t[0] | ((t[1] & 0x0F) << 8)
        w = (t[2] << 4) | ((t[1] & 0xF0) >> 4)
        return r/10-273 if r < 5000 else 0, w/10-273 if w < 5000 else 0, t[5], t, a
    return 0, 0, 0, None, None

try:
    cycle = 0
    while True:
        t0 = time.time()
        for si in range(3):
            room, water, b5, tdata, adata = burst(si)
            n = NAMES[si]
            tac = ' '.join(f'{b:02x}' for b in adata) if adata is not None else '?'
            sys.stdout.write(f"\r{cycle}.{si} {n} | Room={room:.1f}°C W={water:.1f}°C b5=0x{b5:02x} AC={tac}  ")
            sys.stdout.flush()
        elapsed = time.time() - t0
        cycle += 1
        print(f"burst={elapsed*1000:.0f}ms")
        time.sleep(max(0, 13.0 - elapsed))

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
