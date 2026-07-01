#!/usr/bin/env python3
"""
Test: break 1.5ms (standard LIN) + 50ms entre trames, cycle complet
"""
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
    ser.reset_input_buffer()
    brk()
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()
    if data is not None:
        cs = calc_cs(data if classic else bytes([raw]) + data)
        ser.write(data + bytes([cs]))
        ser.flush()
        ser.reset_input_buffer()

def rframe(pid):
    ser.reset_input_buffer()
    brk()
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()
    resp = ser.read(100)
    for i in range(len(resp)-5):
        if resp[i] == 0x55 and resp[i+1] == raw:
            return resp[i+2:i+10]
    if len(resp) >= 12:
        return resp[-9:-1]
    return None

# Wake-up: 1ms pulse + 1.7s idle (comme iNetX)
# cycle total ~12s comme iNetX
ser.send_break(0.001)
time.sleep(10.0)
ser.reset_input_buffer()

cmdstat = bytes.fromhex('aa aa aa 00 09 a2 e0 0f')  # vent 10
TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
TX = [TX_A, TX_B, TX_C]
NAMES = ['A', 'B', 'C']

try:
    cycle = 0
    CYCLE_S = 12.0
    while True:
        t_cycle_start = time.time()

        # 1ms pulse + 1.7s idle
        ser.send_break(0.001)
        time.sleep(1.7)
        ser.reset_input_buffer()

        for si in range(3):
            wframe(0x3C, TX[si], classic=True)
            rd = rframe(0x3D)
            wframe(0x20, cmdstat)
            td = rframe(0x21)
            ad = rframe(0x22)
            rframe(0x0A)
            rframe(0x1F)

            if td:
                r = td[0] | ((td[1] & 0x0F) << 8)
                w = (td[2] << 4) | ((td[1] & 0xF0) >> 4)
                room = r/10-273 if r < 5000 else 0
            else:
                room = 0
            b5 = td[5] if td else 0
            s2m = rd.hex()[:16] if rd else '---'
            sys.stdout.write(f"\r{cycle}.{si} {NAMES[si]} | R={room:.1f} b5=0x{b5:02x} S2M={s2m}  ")
            sys.stdout.flush()

        elapsed = time.time() - t_cycle_start
        remaining = CYCLE_S - elapsed
        if remaining > 0:
            time.sleep(remaining)
        print(f"  cycle={elapsed:.1f}s")

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
