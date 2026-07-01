#!/usr/bin/env python3
"""
Truma LIN Bus Master
Remplace l'iNetX comme maître LIN.
Protocol: break 1.5ms, 50ms inter-frame
Idle: 3 sub-cycles every ~12s
Active: 2 sub-cycles every ~700ms
"""
import serial, time, sys, fcntl, termios, json, traceback
import paho.mqtt.client as mqtt

UART = '/dev/serial0'
BAUD = 9600

MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = 'service/truma'

TIOCSBRK = 0x5427
TIOCCBRK = 0x5428

PID_RAW = {
    0x20: 0x20, 0x21: 0x61, 0x22: 0xE2,
    0x0A: 0xCA, 0x1F: 0x1F,
    0x3C: 0x3C, 0x3D: 0x7D,
}

TX_A = bytes.fromhex('7f 06 b2 00 17 46 20 03')
TX_B = bytes.fromhex('01 06 b8 20 03 01 00 ff')
TX_C = bytes.fromhex('7f 06 b2 23 17 46 20 03')
TX_LIST = [TX_A, TX_B, TX_C]

def calc_cs(data):
    cs = sum(data)
    while cs > 0xFF:
        cs = (cs & 0xFF) + (cs >> 8)
    return (~cs) & 0xFF or 0xFF

class TrumaMaster:
    def __init__(self):
        self.ser = serial.Serial(UART, BAUD, timeout=0.050)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.cmd_room = 0
        self.cmd_water_mode = 'off'
        self.cmd_mode = 'off'
        self.cmd_energy = 'elec'
        self.cmd_power = 900
        self.cmd_vent = 0

        self.status_room = 0.0
        self.status_water = -1.0
        self._room_buf = []
        self._water_buf = []
        self.status_b5 = 0
        self.status_heartbeat = ''
        self.status_ac = ''
        self.status_mains = True

        self.active = False
        self.last_pub = 0

        self.mqtt = None
        self._ready = False
        self._setup_mqtt()
        for t in ('air_mode', 'room_temp', 'water_mode', 'ventilation', 'energy', 'power'):
            val = 'off' if t in ('air_mode', 'water_mode', 'room_temp') else ('0' if t == 'ventilation' else ('elec' if t == 'energy' else '900'))
            self.mqtt.publish(f'{MQTT_TOPIC}/cmd/{t}', val, retain=True)
        time.sleep(0.3)
        self._ready = True
        self.mqtt.subscribe(f'{MQTT_TOPIC}/cmd/#')
        self._wake_truma()

    def brk(self):
        fcntl.ioctl(self.ser.fd, TIOCSBRK)
        time.sleep(0.0015)
        fcntl.ioctl(self.ser.fd, TIOCCBRK)
        time.sleep(0.002)

    def wframe(self, pid, data=None, classic=False):
        self.ser.reset_input_buffer()
        self.brk()
        raw = PID_RAW[pid]
        self.ser.write(bytes([0x55, raw]))
        self.ser.flush()
        if data is not None:
            cs = calc_cs(data if classic else bytes([raw]) + data)
            self.ser.write(data + bytes([cs]))
            self.ser.flush()
            self.ser.reset_input_buffer()

    def rframe(self, pid):
        self.ser.reset_input_buffer()
        self.brk()
        raw = PID_RAW[pid]
        self.ser.write(bytes([0x55, raw]))
        self.ser.flush()
        resp = self.ser.read(100)
        for i in range(len(resp) - 5):
            if resp[i] == 0x55 and resp[i+1] == raw:
                return resp[i+2:i+10]
        if len(resp) >= 12:
            return resp[-9:-1]
        return None

    def _wake_truma(self):
        self.brk()
        self.ser.write(bytes([0x55, 0xFF]))
        self.ser.flush()
        time.sleep(1.0)
        self.ser.reset_input_buffer()

    def _build_cmdstat(self):
        if self.cmd_room <= 0:
            room_raw = 0xAAA
        else:
            room_raw = int((self.cmd_room + 273) * 10) & 0xFFF

        mode_map = {'off': 0x00, 'eco': 0x0B, 'high': 0x0D}
        if self.cmd_mode == 'vent':
            vent_mode = self.cmd_vent
        else:
            vent_mode = mode_map.get(self.cmd_mode, 0x00)

        energy_map = {'gas': 0x01, 'elec': 0x02, 'mix': 0x03}
        en2 = energy_map.get(self.cmd_energy, 0x02)
        emix = 0x00 if self.cmd_energy == 'elec' else 0xFA

        water_map = {'off': 0xAAA, 'eco': 0xC30, 'comfort': 0xCD0, 'hot': 0xD00}
        water_targets = {'eco': 39, 'comfort': 55, 'hot': 60}
        water_raw = water_map.get(self.cmd_water_mode, 0xAAA)
        if self.cmd_water_mode != 'off' and self.status_water > 0:
            target = water_targets.get(self.cmd_water_mode, 99)
            if self.status_water >= target - 4:
                water_raw = 0xAAA

        power_byte = 0x12 if self.cmd_power == 1800 else 0x09

        b0 = room_raw & 0xFF
        b1 = ((room_raw >> 8) & 0x0F) | ((water_raw & 0x0F) << 4)
        b2 = (water_raw >> 4) & 0xFF
        return bytes([b0, b1, b2, emix, power_byte, (vent_mode << 4) | en2, 0xE0, 0x0F])

    def _run_subcycle(self, si, cmd):
        self.wframe(0x3C, TX_LIST[si], classic=True)
        rd = self.rframe(0x3D)
        self.wframe(0x20, cmd)
        td = self.rframe(0x21)
        ad = self.rframe(0x22)
        self.rframe(0x0A)
        self.rframe(0x1F)

        if td:
            r = td[0] | ((td[1] & 0x0F) << 8)
            w = (td[2] << 4) | ((td[1] & 0xF0) >> 4)
            if 2630 < r < 3330:
                self._room_buf.append((r / 10.0) - 273)
                if len(self._room_buf) > 3:
                    self._room_buf.pop(0)
                self.status_room = sorted(self._room_buf)[len(self._room_buf)//2]
            if 2730 < w < 3730:
                self._water_buf.append((w / 10.0) - 273)
                if len(self._water_buf) > 3:
                    self._water_buf.pop(0)
                self.status_water = sorted(self._water_buf)[len(self._water_buf)//2]
            self.status_b5 = td[5]

        if rd:
            self.status_heartbeat = rd.hex()[:16]
        if ad:
            self.status_ac = ad.hex()[:16]
            self.status_mains = (ad[1] & 0x20) != 0

    def _publish_status(self, master=None):
        if not self.mqtt:
            return
        now = time.time()
        if now - self.last_pub < 2.0:
            return
        self.last_pub = now
        try:
            if self._room_buf and -20 < self.status_room < 60:
                self.mqtt.publish(f'{MQTT_TOPIC}/room_temp', f'{self.status_room:.1f}', retain=True)
            if self._water_buf and -10 < self.status_water < 100:
                self.mqtt.publish(f'{MQTT_TOPIC}/water_temp', f'{self.status_water:.1f}', retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/ventilation', str(self.cmd_vent), retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/air_mode', self.cmd_mode, retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/energy', self.cmd_energy, retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/power', str(self.cmd_power), retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/water_mode', self.cmd_water_mode, retain=True)
            water_targets = {'eco': 39, 'comfort': 55, 'hot': 60}
            wh_on = self.cmd_water_mode != 'off' and self.status_water < water_targets.get(self.cmd_water_mode, 99) - 4
            self.mqtt.publish(f'{MQTT_TOPIC}/water_heating', 'ON' if wh_on else 'OFF', retain=True)
            self.mqtt.publish(f'{MQTT_TOPIC}/acin', 'ON' if self.status_mains else 'OFF', retain=True)
            if master:
                self.mqtt.publish(f'{MQTT_TOPIC}/master', master, retain=True)
        except:
            pass

    def _on_connect(self, client, userdata, flags, rc):
        print(f'MQTT connecté (rc={rc})')

    def _on_message(self, client, userdata, msg):
        if not self._ready:
            return
        topic = msg.topic
        payload = msg.payload.decode().strip()
        changed = False

        try:
            if topic.endswith('/air_mode'):
                if payload in ('off', 'eco', 'high', 'vent'):
                    self.cmd_mode = payload
                    if payload == 'off':
                        self.cmd_room = 0
                        self.cmd_water_mode = 'off'
                        self.cmd_vent = 0
                    changed = True
            elif topic.endswith('/room_temp'):
                if payload.lower() == 'off':
                    self.cmd_room = 0
                else:
                    self.cmd_room = float(payload)
                changed = True
            elif topic.endswith('/water_mode'):
                if payload in ('off', 'eco', 'comfort', 'hot'):
                    self.cmd_water_mode = payload
                    changed = True
            elif topic.endswith('/ventilation'):
                self.cmd_vent = int(payload)
                if self.cmd_vent > 0:
                    self.cmd_mode = 'vent'
                changed = True
            elif topic.endswith('/energy'):
                if payload in ('elec', 'gas', 'mix'):
                    self.cmd_energy = payload
                    changed = True
            elif topic.endswith('/power'):
                if payload in ('900', '1800'):
                    self.cmd_power = int(payload)
                    changed = True
        except:
            return

        if changed:
            self.active = self.cmd_mode != 'off' or self.cmd_room > 0 or self.cmd_water_mode != 'off' or self.cmd_vent > 0
            print(f'  Cde: mode={self.cmd_mode} room={self.cmd_room} eau={self.cmd_water_mode} vent={self.cmd_vent} en={self.cmd_energy} W={self.cmd_power} actif={self.active}')

    def _setup_mqtt(self):
        try:
            client = mqtt.Client()
            client.on_connect = self._on_connect
            client.on_message = self._on_message
            client.connect(MQTT_BROKER, MQTT_PORT, 60)
            client.loop_start()
            self.mqtt = client
            print('MQTT OK')
        except Exception as e:
            print(f'MQTT indisponible: {e}')
            self.mqtt = None

    def _detect_bus_activity(self, initial=False):
        self.ser.reset_input_buffer()
        n = 200 if initial else 5
        for _ in range(n):
            time.sleep(0.010)
            if self.ser.in_waiting > 0:
                return True
        return False

    def _decode_and_publish(self, pid_mapped, frame):
        if pid_mapped == 0x20:
            emix = frame[3]
            power_byte = frame[4]
            mode_byte = frame[5]
            en2 = mode_byte & 0x0F
            vent_mode = (mode_byte >> 4) & 0x0F
            energy_map_r = {1: 'gas', 2: 'elec', 3: 'mix'}
            self.cmd_energy = energy_map_r.get(en2, 'elec')
            if vent_mode <= 10:
                self.cmd_vent = vent_mode
                if vent_mode > 0:
                    self.cmd_mode = 'vent'
            elif vent_mode == 0x0B:
                self.cmd_mode = 'eco'
                self.cmd_vent = 0
            elif vent_mode == 0x0D:
                self.cmd_mode = 'high'
                self.cmd_vent = 0
            else:
                self.cmd_mode = 'off'
                self.cmd_vent = 0
            water_val = (frame[2] << 4) | ((frame[1] & 0xF0) >> 4)
            water_map_r = {0xAAA: 'off', 0xC30: 'eco', 0xCD0: 'comfort', 0xD00: 'hot'}
            self.cmd_water_mode = 'off'
            for val, name in water_map_r.items():
                if abs(water_val - val) < 16:
                    self.cmd_water_mode = name
                    break
            self.cmd_power = 1800 if power_byte == 0x12 else 900

        elif pid_mapped == 0x21:
            r = frame[0] | ((frame[1] & 0x0F) << 8)
            w = (frame[2] << 4) | ((frame[1] & 0xF0) >> 4)
            if 2630 < r < 3330:
                self._room_buf.append((r / 10.0) - 273)
                if len(self._room_buf) > 3:
                    self._room_buf.pop(0)
                self.status_room = sorted(self._room_buf)[len(self._room_buf)//2]
            if 2730 < w < 3730:
                self._water_buf.append((w / 10.0) - 273)
                if len(self._water_buf) > 3:
                    self._water_buf.pop(0)
                self.status_water = sorted(self._water_buf)[len(self._water_buf)//2]
            self.status_b5 = frame[5]

        elif pid_mapped == 0x22:
            self.status_ac = frame.hex()[:16]
            self.status_mains = (frame[1] & 0x20) != 0
        elif pid_mapped == 0x3D:
            self.status_heartbeat = frame.hex()[:16]

        self._publish_status()

    def _listen_decode_frame(self, data):
        for i in range(len(data) - 5):
            if data[i] != 0x55:
                continue
            pid = data[i+1]
            if len(data) < i + 11:
                continue
            frame = data[i+2:i+10]
            pid_mapped = None
            for k, v in PID_RAW.items():
                if v == pid:
                    pid_mapped = k
                    break
            if pid_mapped is not None:
                self._decode_and_publish(pid_mapped, frame)

    def _listener_loop(self):
        print('  → Mode écoute passive (iNetX)')
        self.last_pub = 0
        self._publish_status(master='inetx')
        last_frame = time.time()
        buf = bytearray()
        while True:
            if self.ser.in_waiting:
                buf.extend(self.ser.read(self.ser.in_waiting))
                last_frame = time.time()
                self._listen_decode_frame(buf)
                if len(buf) > 512:
                    buf = buf[-256:]
                self._publish_status(master='inetx')
            elif time.time() - last_frame > 30.0:
                print('  → Bus silencieux 30s, reprise en mode master')
                return
            else:
                time.sleep(0.010)

    def run(self):
        cmd = self._build_cmdstat()
        print(f'Démarrage LIN master. Mode={"idle" if not self.active else "actif"}')
        print('  → Détection activité bus...')
        if self._detect_bus_activity(initial=True):
            print('  → Bus occupé au démarrage (iNetX présent)')
            self._listener_loop()
            self.last_pub = 0
            cmd = self._build_cmdstat()
            self._publish_status(master='pi')
            self.ser.reset_input_buffer()

        start = time.time()
        last_status_print = 0
        listener_mode = False

        try:
            while True:
                if listener_mode:
                    self._listener_loop()
                    listener_mode = False
                    self.last_pub = 0
                    cmd = self._build_cmdstat()
                    self._publish_status(master='pi')
                    self.ser.reset_input_buffer()
                    start = time.time()

                if self._detect_bus_activity():
                    listener_mode = True
                    self._publish_status(master='inetx')
                    continue

                t0 = time.time()

                if self.active:
                    n_sub = 2
                else:
                    n_sub = 3
                    fcntl.ioctl(self.ser.fd, TIOCSBRK)
                    time.sleep(0.001)
                    fcntl.ioctl(self.ser.fd, TIOCCBRK)
                    time.sleep(1.7)
                    self.ser.reset_input_buffer()

                for si in range(n_sub):
                    self._run_subcycle(si, cmd)


                elapsed = time.time() - t0

                now = time.time()
                if now - last_status_print > 5.0:
                    hb = self.status_heartbeat or '---'
                    ac = self.status_ac or '---'
                    print(f'\r[{(now-start)/60:.0f}m] Room={self.status_room:.1f}°C Eau={self.status_water:.1f}°C b5=0x{self.status_b5:02x} HB={hb} AC={ac} mode={self.cmd_mode} vent={self.cmd_vent} eau={self.cmd_water_mode} {self.cmd_power}W    ')
                    last_status_print = now

                self._publish_status(master='pi')

                cycle_s = 12.0 if not self.active else 0.7
                remaining = cycle_s - elapsed
                if remaining > 0:
                    time.sleep(remaining)

                cmd = self._build_cmdstat()
                if self.active and self.cmd_mode == 'off' and self.cmd_room <= 0 and self.cmd_water_mode == 'off' and self.cmd_vent <= 0:
                    self.active = False
                    print('  → Passage en mode idle')

        except KeyboardInterrupt:
            print('\nArrêt')
        except:
            traceback.print_exc()
        finally:
            if self.mqtt:
                self.mqtt.loop_stop()
                self.mqtt.disconnect()
            self.ser.close()

if __name__ == '__main__':
    m = TrumaMaster()
    m.run()
