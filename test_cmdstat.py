#!/usr/bin/env python3
"""
Test : envoyer le CmdStat exact de l'iNetX pour climat ambiant chauffage 28°C eco.
Vérifier si la Truma exécute la commande.
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

def send_frame(pid, tx_data=None):
    ser.reset_input_buffer()
    ser.send_break(0.020)
    time.sleep(0.005)
    raw = PID_RAW[pid]
    ser.write(bytes([0x55, raw]))
    ser.flush()

    if tx_data is not None:
        cs = calc_cs(bytes([raw]) + tx_data)
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
ser.send_break(0.020)
time.sleep(0.005)
ser.write(bytes([0x55, 0xFF]))
time.sleep(1.0)
ser.reset_input_buffer()

# Initial read
print("\n=== État initial ===")
data, cs = send_frame(0x21)
if data:
    r = data[0] | ((data[1] & 0x0F) << 8)
    w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
    print(f"  Temp: Room={r/10-273:.1f}°C Water={w/10-273:.1f}°C  b5=0x{data[5]:02x}")
data, cs = send_frame(0x22)
if data:
    print(f"  AC:   {' '.join(f'{b:02x}' for b in data)}  b0=0x{data[0]:02x}")

# CmdStat: 28°C eco elec (exact iNetX pattern)
cmdstat = bytes.fromhex('c2 ab aa 00 09 b2 e0 0f')
cs = calc_cs(bytes([0x20]) + cmdstat)
print(f"\n=== Envoi CmdStat: 28°C eco (CS=0x{cs:02x}) ===")
print(f"    {' '.join(f'{b:02x}' for b in cmdstat)}")

try:
    cycle = 0
    while True:
        t0 = time.time()

        send_frame(0x20, cmdstat)

        data, cs = send_frame(0x21)
        if data:
            r = data[0] | ((data[1] & 0x0F) << 8)
            w = (data[2] << 4) | ((data[1] & 0xF0) >> 4)
            room = r/10-273 if r < 5000 else 0
            water = w/10-273 if w < 5000 else 0
            b5 = data[5]
            ac = ' '.join(f'{b:02x}' for b in data)

        data2, cs2 = send_frame(0x22)
        ac_b0 = data2[0] if data2 else 0

        elapsed = time.time() - t0
        cycle += 1
        sys.stdout.write(f"\rCycle {cycle:3d} | Room={room:.1f}°C Water={water:.1f}°C  Temp[5]=0x{b5:02x}  AC b0=0x{ac_b0:02x}  {elapsed*1000:.0f}ms   ")
        sys.stdout.flush()

        time.sleep(max(0, 2.0 - elapsed))

except KeyboardInterrupt:
    print("\nArrêt.")
finally:
    ser.close()
