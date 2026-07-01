#!/usr/bin/env python3
"""
Test: rafale serrée comme l'iNetX, break ioctl 13ms
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

def fast_write(pid, data=None, classic=False):
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
        time.sleep(0.010)
        ser.reset_input_buffer()

def fast_read(pid):
    ser.reset_input_buffer()
    brk(13)
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()
    time.sleep(0.050)
    resp = ser.read(100)
    for i in range(len(resp)-5):
        if resp[i] == 0x55 and resp[i+1] == raw:
            return resp[i+2:i+10]
    if len(resp) >= 12:
        return resp[-9:-1]
    return None

# Wake-up
brk(20)
ser.write(bytes([0x55, 0xFF]))
time.sleep(1.0)
ser.reset_input_buffer()

cmdstat = bytes.fromhex('c2 ab aa 00 09 b2 e0 0f')
TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
TX = [TX_A, TX_B, TX_C]
NAMES = ['A', 'B', 'C']

try:
    cycle = 0
    while True:
        t0 = time.time()
        for si in range(3):
            fast_write(0x3C, TX[si], classic=True)
            rd = fast_read(0x3D)
            fast_write(0x20, cmdstat)
            td = fast_read(0x21)
            ad = fast_read(0x22)
            fast_read(0x0A)
            fast_read(0x1F)

            if td:
                r = td[0] | ((td[1] & 0x0F) << 8)
                w = (td[2] << 4) | ((td[1] & 0xF0) >> 4)
                room = r/10-273 if r < 5000 else 0
                water = w/10-273 if w < 5000 else 0
            else:
                room = water = 0

            b5 = td[5] if td else 0
            s2m = rd.hex()[:16] if rd else '---'
            sys.stdout.write(f"\r{cycle}.{si} {NAMES[si]} | R={room:.1f} W={water:.1f} b5=0x{b5:02x} S2M={s2m}  ")
            sys.stdout.flush()

        t1 = time.time()
        print(f"burst={t1-t0:.2f}s  idle=13s")
        time.sleep(max(0, 13.0 - (t1 - t0)))

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
