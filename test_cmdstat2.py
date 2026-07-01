#!/usr/bin/env python3
"""
Test: break via baud rate trick (600 baud 0x00 = 15ms break)
puis CmdStat 28°C eco.
"""
import serial, time, sys

UART = '/dev/serial0'
ser = serial.Serial(UART, 9600, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()

PID_RAW = {0x20: 0x20, 0x21: 0x61, 0x22: 0xE2}

def calc_cs(data):
    cs = sum(data)
    while cs > 0xFF: cs = (cs & 0xFF) + (cs >> 8)
    return (~cs) & 0xFF or 0xFF

def send_break_15ms():
    ser.baudrate = 600      # 1 bit = 1.667ms
    ser.write(b'\x00')      # start(0) + 8x0 + stop(1) = 15ms dominant
    ser.flush()             # wait for transmission to finish
    ser.baudrate = 9600
    time.sleep(0.003)

def send_header(pid_raw):
    ser.reset_input_buffer()
    send_break_15ms()
    ser.write(bytes([0x55, pid_raw]))
    ser.flush()

def send_cmd(pid, data):
    raw = PID_RAW[pid]
    send_header(raw)
    cs = calc_cs(bytes([raw]) + data)
    time.sleep(0.003)
    ser.write(data + bytes([cs]))
    ser.flush()
    time.sleep(0.030)
    ser.reset_input_buffer()

def read_query(pid):
    raw = PID_RAW[pid]
    send_header(raw)
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
print("Wake-up (baud trick)...")
ser.baudrate = 600
ser.write(b'\x00')
ser.flush()
ser.baudrate = 9600
time.sleep(0.005)
ser.reset_input_buffer()
time.sleep(1.0)

# Initial read
print("\n=== État initial ===")
data, cs = read_query(0x21)
if data:
    r = data[0] | ((data[1] & 0x0F) << 8)
    w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
    print(f"  Temp: Room={r/10-273:.1f}°C Water={w/10-273:.1f}°C  b5=0x{data[5]:02x}")
    print(f"  Data: {' '.join(f'{b:02x}' for b in data)}")
data, cs = read_query(0x22)
if data:
    print(f"  AC:   {' '.join(f'{b:02x}' for b in data)}  CS=0x{cs:02x}")

# CmdStat 28°C eco
cmdstat = bytes.fromhex('c2 ab aa 00 09 b2 e0 0f')
print(f"\n=== CmdStat 28°C eco (baud trick break) ===")

try:
    cycle = 0
    while True:
        t0 = time.time()

        send_cmd(0x20, cmdstat)

        data, cs = read_query(0x21)
        if data:
            r = data[0] | ((data[1] & 0x0F) << 8)
            w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
            room = r/10-273 if r < 5000 else 0
            water = w/10-273 if w < 5000 else 0

        data2, cs2 = read_query(0x22)
        ac = ' '.join(f'{b:02x}' for b in data2) if data2 else '?'

        elapsed = time.time() - t0
        cycle += 1
        sys.stdout.write(f"\rCycle {cycle:3d} | Room={room:.1f}°C Water={water:.1f}°C  AC={ac}  {elapsed*1000:.0f}ms    ")
        sys.stdout.flush()

        time.sleep(max(0, 2.0 - elapsed))

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
