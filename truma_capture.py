#!/usr/bin/env python3
"""
Capture le bus LIN Truma avec horodatage et export JSON
Utilisation : 
  python3 truma_capture.py              # capture en direct
  python3 truma_capture.py --log base   # sauvegarde dans base.log
  python3 truma_capture.py --diff a.log b.log   # compare deux logs
"""

import serial
import time
import sys
import os
import json
from datetime import datetime

UART_PORT = '/dev/serial0'
BAUD_RATE = 9600

PID_FILTER = None  # mettre un PID pour filtrer, ex: 0x20


def calc_checksum(data):
    cs = sum(data)
    while cs > 0xFF:
        cs = (cs & 0xFF) + (cs >> 8)
    cs = (~cs) & 0xFF
    return 0xFF if cs == 0 else cs


def timestamp():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def capture_lin(duration=30, logfile=None):
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    
    buffer = bytearray()
    start = time.time()
    frames = []
    
    out = open(logfile, 'w') if logfile else sys.stdout
    
    try:
        if logfile:
            out.write(f"# Capture démarrée à {datetime.now().isoformat()}\n")
            out.write(f"# Format: timestamp_abs timestamp_rel PID_raw PID payload_hex\n")
        
        while time.time() - start < duration:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                while len(buffer) >= 3:
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
                        pid_raw = msg[2]
                        pid = pid_raw & 0x3F
                        
                        if PID_FILTER and pid != PID_FILTER:
                            continue
                        
                        payload = msg[3:11]
                        cs = msg[11]
                        
                        # vérif checksum
                        if pid in [0x3C, 0x3D]:
                            cs_ok = calc_checksum(payload) == cs
                        else:
                            cs_ok = calc_checksum(bytes([pid_raw]) + payload) == cs
                        
                        ts_abs = timestamp()
                        ts_rel = f"{time.time()-start:.3f}"
                        raw_hex = ' '.join(f'{b:02x}' for b in msg)
                        frame = {
                            'ts_abs': ts_abs,
                            'ts_rel': ts_rel,
                            'pid_raw': f'0x{pid_raw:02X}',
                            'pid': f'0x{pid:02X}',
                            'payload': ' '.join(f'{b:02x}' for b in payload),
                            'cs': f'0x{cs:02X}',
                            'cs_ok': cs_ok,
                            'raw': raw_hex,
                        }
                        frames.append(frame)
                        
                        prefix = f"[{ts_abs}] +{ts_rel}s"
                        cs_mark = '✓' if cs_ok else '✗'
                        out.write(f"{prefix}  PID {frame['pid']} (raw {frame['pid_raw']}) [{cs_mark}]  {frame['payload']}\n")
                        if logfile:
                            out.flush()
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    
    ser.close()
    if logfile:
        out.close()
    
    return frames


def diff_logs(f1, f2):
    """Compare deux fichiers de log et montre les différences"""
    frames1 = []
    frames2 = []
    
    with open(f1) as f:
        for line in f:
            if line.startswith('#'):
                continue
            if '[✓]' in line or '[✗]' in line:
                frames1.append(line.strip())
    
    with open(f2) as f:
        for line in f:
            if line.startswith('#'):
                continue
            if '[✓]' in line or '[✗]' in line:
                frames2.append(line.strip())
    
    # Comparer cycle par cycle
    # Un cycle = un ensemble de PIDs qui se répète
    # On cherche le pattern: 20, 21, 22, 0A, 1F, 3C, 3D
    # Extract just PID and payload
    def extract_pid_data(lines):
        result = []
        for line in lines:
            parts = line.split('PID')
            if len(parts) >= 2:
                pid_part = parts[1].strip().split()[0]
                payload_part = line.split(']')[1].strip() if ']' in line else ''
                result.append((pid_part, payload_part))
        return result
    
    d1 = extract_pid_data(frames1)
    d2 = extract_pid_data(frames2)
    
    print(f"Fichier 1: {f1} ({len(d1)} trames)")
    print(f"Fichier 2: {f2} ({len(d2)} trames)")
    print()
    
    # Trouver les différences
    for i, ((pid1, p1), (pid2, p2)) in enumerate(zip(d1, d2)):
        if pid1 == pid2:
            # Même PID, comparer payload
            # Extraire juste les octets de données
            payload1 = p1.strip()
            payload2 = p2.strip()
            if payload1 != payload2:
                print(f"≠ Trame #{i}  {pid1}")
                print(f"  Avant : {payload1}")
                print(f"  Après : {payload2}")
                # Afficher octet par octet
                b1 = payload1.split()
                b2 = payload2.split()
                if len(b1) == len(b2):
                    for j, (a, b) in enumerate(zip(b1, b2)):
                        mark = " ←" if a != b else ""
                        print(f"    [{j}] 0x{a} → 0x{b}{mark}")
                print()
        else:
            print(f"≠ Trame #{i}  {pid1} → {pid2} (PID changé!)")
    
    if len(d1) != len(d2):
        print(f"⚠ Nombre de trames différent: {len(d1)} vs {len(d2)}")


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--diff':
        diff_logs(sys.argv[2], sys.argv[3])
        return
    
    logfile = None
    duration = 30
    
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '--log' and i+1 < len(args):
            logfile = args[i+1]
            i += 2
        elif args[i] == '--duration' and i+1 < len(args):
            duration = int(args[i+1])
            i += 2
        elif args[i] == '--pid' and i+1 < len(args):
            global PID_FILTER
            PID_FILTER = int(args[i+1], 16)
            i += 2
        else:
            i += 1
    
    if logfile:
        print(f"Capture {duration}s → {logfile}")
        print(f"Appuie sur Ctrl+C pour arrêter avant la fin")
        capture_lin(duration=duration, logfile=logfile)
        print(f"Sauvegardé dans {logfile}")
    else:
        capture_lin(duration=duration)


if __name__ == '__main__':
    main()
