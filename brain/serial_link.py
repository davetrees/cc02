"""Serial link thread: ESP32-S3 over USB CDC.
Handles absent device with a 2s reconnect loop; sends CMD@50Hz + DISP@10Hz always.
"""
import glob
import socket
import struct
import threading
import time

import serial

import protocol as P


def find_port():
    ports = sorted(glob.glob('/dev/serial/by-id/*'))
    if ports:
        return ports[0]
    ports = glob.glob('/dev/ttyACM0')
    return ports[0] if ports else None


def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '0.0.0.0'


class SerialLink(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True, name='serial')
        self.state = state
        self.ser = None
        self._buf = bytearray()
        self._telem_count = 0
        self._hz_t0 = time.monotonic()
        self._ip = get_ip()
        self._ip_t = time.monotonic()

    def run(self):
        st = self.state
        while st.running:
            port = find_port()
            if port is None:
                st.link_connected = False
                st.log('serial: no device, retrying in 2s')
                time.sleep(2.0)
                continue
            try:
                self.ser = serial.Serial(port, 115200, timeout=0)
                st.log(f'serial: opened {port}')
                st.link_connected = True
                self._loop()
            except (serial.SerialException, OSError) as e:
                st.log(f'serial: error {e}; reconnect in 2s')
            finally:
                st.link_connected = False
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
            time.sleep(2.0)

    def _loop(self):
        st = self.state
        tick = 0
        next_t = time.monotonic()
        while st.running:
            next_t += 0.02  # 50 Hz
            # read + parse everything available
            data = self.ser.read(4096)
            if data:
                self._buf.extend(data)
                self._parse_buf()
            if len(self._buf) > 8192:
                del self._buf[:4096]
            # send CMD every tick (50Hz), DISP every 5th (10Hz)
            self.ser.write(P.build_cmd(*st.cmd_tuple()))
            if tick % 5 == 0:
                self.ser.write(self._disp())
            tick += 1
            # telemetry hz once a second
            now = time.monotonic()
            if now - self._hz_t0 >= 1.0:
                st.telem_hz = self._telem_count / (now - self._hz_t0)
                self._telem_count = 0
                self._hz_t0 = now
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()

    def _disp(self):
        st = self.state
        now = time.monotonic()
        if now - self._ip_t > 30:
            self._ip = get_ip()
            self._ip_t = now
        flags = P.DF_COLLISION if st.collision else 0
        # LEVEL/ZERO: hold the cal-request flag for ~1s after the panel asks
        if st.cal_request_time and now - st.cal_request_time < 1.0:
            flags |= P.DF_CAL_REQUEST
        return P.build_disp(st.ws_clients, round(st.yolo_fps), st.det_count,
                            flags, round(st.telem_hz), self._ip)

    def _parse_buf(self):
        st = self.state
        while True:
            idx = self._buf.find(b'\x00')
            if idx < 0:
                return
            chunk = bytes(self._buf[:idx])
            del self._buf[:idx + 1]
            fr = P.parse_frame(chunk)
            if fr is None:
                continue
            ftype, payload = fr
            if ftype == P.TELEM:
                t = P.parse_telem(payload)
                if t is not None:
                    st.telem = t
                    st.last_telem_time = time.monotonic()
                    self._telem_count += 1
            elif ftype == P.MODEREQ and len(payload) >= 1:
                self._modereq(payload[0])

    def _modereq(self, req):
        st = self.state
        if req == 1:      # mode_cycle MANUAL->ASSIST->AUTO->MANUAL
            if st.mode in (P.MODE_MANUAL, P.MODE_ASSIST, P.MODE_AUTO):
                st.set_mode((st.mode + 1) % 3)
            else:
                st.set_mode(P.MODE_MANUAL)
        elif req == 2:    # rth
            st.set_mode(P.MODE_RTH)
        elif req == 3:
            st.estop = True
            st.log('MODEREQ: ESTOP ON')
        elif req == 4:
            st.estop = False
            st.log('MODEREQ: ESTOP OFF')
