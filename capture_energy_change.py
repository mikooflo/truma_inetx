import serial, time, sys

ser = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(0.3)
ser.reset_input_buffer()
buffer = bytearray()
start = time.time()
last_state = {}
samples = []

print("Capture en cours pour 25s... bascule 900W→1800W maintenant !")
while time.time() - start < 25:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        buffer.extend(data)
        while len(buffer) >= 12:
            found = -1
            for i in range(len(buffer)-1):
                if buffer[i] == 0x00 and buffer[i+1] == 0x55:
                    found = i
                    break
            if found == -1:
                buffer.clear()
                break
            if found > 0:
                buffer = buffer[found:]
            if len(buffer) >= 12:
                msg = buffer[:12]
                buffer = buffer[12:]
                pid = msg[2] & 0x3F
                payload = bytes(msg[3:11])
                t = time.time() - start
                samples.append((t, pid, payload))
    else:
        time.sleep(0.01)

ser.close()

# Show changes during capture
last = {}
print("\n=== CHANGEMENTS DÉTECTÉS ===")
for t, pid, payload in samples:
    prev = last.get(pid)
    if prev is not None and prev != payload:
        pname = {0x20:"CmdStat",0x21:"Temp",0x22:"AC",0x0A:"Unk0A",0x1F:"Unk1F",0x3C:"TxM→S",0x3D:"TxS→M"}.get(pid, f"PID{pid:02X}")
        print(f"  {t:6.3f}s {pname} : {' '.join(f'{b:02x}' for b in prev)} → {' '.join(f'{b:02x}' for b in payload)}")
    last[pid] = payload

print("\n=== ÉTAT FINAL CmdStat ===")
p = last.get(0x20)
if p:
    print(f"  PID 0x20: {' '.join(f'{b:02x}' for b in p)}")
    room_temp = (p[0] | ((p[1] & 0x0F) << 8)) / 10.0 - 273
    water_temp = ((p[2] << 4) | ((p[1] & 0xF0) >> 4)) / 10.0 - 273
    vent_mode = p[5] >> 4
    energy2 = p[5] & 0x0F
    print(f"  Temp chauffage: {room_temp:.1f}°C")
    print(f"  Temp eau: {water_temp:.1f}°C")
    print(f"  byte[3]=0x{p[3]:02x} byte[4]=0x{p[4]:02x} byte[5]=0x{p[5]:02x} (mode=0x{vent_mode:x}, energy2=0x{energy2:x})")
