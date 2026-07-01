#!/usr/bin/env python3
"""
Capture les trames du bus LIN Truma (iNetX + Truma)
Synchronisation correcte : 0x00 0x55 = start of frame
"""
import serial, time, sys

UART = '/dev/serial0'
BAUD = 9600

PID_NAMES = {
    0x20: "CmdStat", 0x21: "Temp", 0x22: "AC",
    0x0A: "Unk0A", 0x1F: "Unk1F",
    0x3C: "TxM2S", 0x3D: "TxS2M",
}

ser = serial.Serial(UART, BAUD, timeout=0.5)
time.sleep(0.3)
ser.reset_input_buffer()

buffer = bytearray()
start = time.time()
frame_count = 0

try:
    while time.time() - start < 180:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

        while len(buffer) >= 13:
            synced = -1
            for i in range(len(buffer) - 12):
                if buffer[i] == 0x00 and buffer[i+1] == 0x55:
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
                frame_count += 1
                pid_raw = msg[2]
                pid = pid_raw & 0x3F
                payload = msg[3:11]
                cs = msg[11]
                pname = PID_NAMES.get(pid, f"0x{pid:02X}")
                t = time.strftime('%H:%M:%S')

                # Affichage court pour l'ecran
                sys.stdout.write(f"[{t}] PID 0x{pid:02X} ({pname:6s})  {''.join(f'{b:02x}' for b in payload)}  cs={cs:02x}\n")
                sys.stdout.flush()
            else:
                break

        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print(f"\n{frame_count} trames capturees")
