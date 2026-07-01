#!/usr/bin/env python3
"""
Test: break via ioctl TIOCSBRK/TIOCCBRK (13ms),
puis cycle complet avec CmdStat 28°C eco.
"""
import serial, time, sys, fcntl, termios, array

UART = '/dev/serial0'
ser = serial.Serial(UART, 9600, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

# ioctl numbers (Linux)
TIOCSBRK = 0x5427  # set break
TIOCCBRK = 0x5428  # clear break

def send_break_ioctl(duration_ms=13):
    fcntl.ioctl(ser.fd, TIOCSBRK)
    time.sleep(duration_ms / 1000.0)
    fcntl.ioctl(ser.fd, TIOCCBRK)
    time.sleep(0.003)

RAW = {0x20: 0x20, 0x21: 0x61, 0x22: 0xE2, 0x3C: 0x3C, 0x3D: 0x7D, 0x0A: 0xCA, 0x1F: 0x1F}

def calc_cs(data):
    cs = sum(data)
    while cs > 0xFF: cs = (cs & 0xFF) + (cs >> 8)
    return (~cs) & 0xFF or 0xFF

def send_frame(pid, tx_data=None, classic_cs=False):
    ser.reset_input_buffer()
    send_break_ioctl(13)
    raw = RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()

    if tx_data is not None:
        cs_data = tx_data if classic_cs else bytes([raw]) + tx_data
        cs = calc_cs(cs_data)
        time.sleep(0.003)
        ser.write(tx_data + bytes([cs]))
        ser.flush()
        time.sleep(0.030)
        ser.reset_input_buffer()
        return tx_data, cs

    time.sleep(0.060)
    resp = ser.read(100)
    idx = -1
    for i in range(len(resp) - 5):
        if resp[i] == 0x55 and resp[i+1] == raw:
            idx = i + 2
            break
    if idx >= 0 and len(resp) >= idx + 9:
        return resp[idx:idx+8], resp[idx+8]
    if len(resp) >= 12:
        return resp[-9:-1], resp[-1]
    return None, None

# Wake-up
print("Wake-up...")
send_break_ioctl(20)
ser.write(bytes([0x55, 0xFF]))
time.sleep(1.0)
ser.reset_input_buffer()

# Initial
print("\n=== Initial ===")
data, cs = send_frame(0x21)
if data:
    r = data[0] | ((data[1] & 0x0F) << 8)
    w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
    print(f"Temp: Room={r/10-273:.1f}°C Water={w/10-273:.1f}°C  b5=0x{data[5]:02x}")

# CmdStat 28°C eco
cmdstat = bytes.fromhex('c2 ab aa 00 09 b2 e0 0f')
TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
subcycles = [(TX_A, 'A'), (TX_B, 'B'), (TX_C, 'C')]

try:
    cycle = 0
    b5 = 0
    while True:
        t0 = time.time()
        sub_tx, sub_name = subcycles[cycle % 3]
        send_frame(0x3C, sub_tx, classic_cs=True)
        resp, cs = send_frame(0x3D)
        s2m = resp.hex() if resp else '---'
        send_frame(0x20, cmdstat)
        data, cs = send_frame(0x21)
        if data:
            r = data[0] | ((data[1] & 0x0F) << 8)
            w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
            room = r/10-273 if r < 5000 else 0
            water = w/10-273 if w < 5000 else 0
            b5 = data[5]
        data2, cs2 = send_frame(0x22)
        ac = ' '.join(f'{b:02x}' for b in data2) if data2 else '?'
        send_frame(0x0A)
        send_frame(0x1F)

        elapsed = time.time() - t0
        cycle += 1
        sys.stdout.write(f"\r{cycle:3d} {sub_name} | Room={room:.1f}°C W={water:.1f}°C b5=0x{b5:02x} AC={ac}  S2M={s2m}  {elapsed*1000:.0f}ms")
        sys.stdout.flush()
        time.sleep(max(0, 2.0 - elapsed))

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
