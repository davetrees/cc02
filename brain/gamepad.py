"""Bluetooth game controllers paired directly to the Pi (evdev).

Supported: Nintendo Switch Pro Controller (hid-nintendo) and SteelSeries
Nimbus / Nimbus+ (BLE HID). Discovery loop every 3s for an evdev device whose
name matches a known controller pattern.
Left stick X -> steer -1..1; right stick vertical (ABS_RY, else ABS_RZ) ->
throttle -1..1 (up=forward). B/A (BTN_SOUTH/BTN_EAST) toggles ESTOP;
PLUS/Menu (BTN_START/BTN_MODE) cycles MANUAL->ASSIST->AUTO.
Dead-man: if events stop for 150ms or the device vanishes, inputs go neutral.
Arbitration with the web panel: most recent input wins (both write the same
user_steer/user_throttle + last_input_time; dead-man only neutralizes when the
gamepad was the last writer).
"""
import re
import threading
import time
from select import select

import protocol as P

DEADZONE = 0.15
DEADMAN_S = 0.15
NAME_RE = re.compile(r'pro controller|nimbus', re.IGNORECASE)


class Gamepad(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True, name='gamepad')
        self.state = state

    def run(self):
        st = self.state
        try:
            import evdev
        except ImportError:
            st.log('gamepad: evdev not installed, controller support disabled')
            return
        announce = True
        while st.running:
            dev = self._find(evdev)
            if dev is None:
                st.gamepad_connected = False
                if announce:
                    st.log('gamepad: no controller (Pro Controller/Nimbus), '
                           'scanning every 3s')
                    announce = False
                time.sleep(3)
                continue
            st.log(f'gamepad: connected "{dev.name}" at {dev.path}')
            st.gamepad_connected = True
            try:
                self._read(dev, evdev)
            except OSError as e:
                st.log(f'gamepad: device lost ({e}), rescanning')
            finally:
                st.gamepad_connected = False
                self._neutral_if_owner()
                try:
                    dev.close()
                except Exception:
                    pass
            announce = True

    def _find(self, evdev):
        try:
            for path in evdev.list_devices():
                try:
                    d = evdev.InputDevice(path)
                except OSError:
                    continue
                if NAME_RE.search(d.name or '') and 'IMU' not in (d.name or ''):
                    # must be the pad node: sticks AND gamepad buttons
                    # (hid-nintendo also exposes an "(IMU)" accel node - skip it)
                    caps = d.capabilities()
                    keys = caps.get(evdev.ecodes.EV_KEY, [])
                    if evdev.ecodes.EV_ABS in caps and (
                            evdev.ecodes.BTN_SOUTH in keys or
                            evdev.ecodes.BTN_GAMEPAD in keys):
                        return d
                d.close()
        except Exception:
            pass
        return None

    def _neutral_if_owner(self):
        st = self.state
        if st.last_input_src == 'gamepad':
            st.user_steer = 0.0
            st.user_throttle = 0.0

    def _read(self, dev, evdev):
        st = self.state
        ec = evdev.ecodes

        def absinfo(code):
            try:
                return dev.absinfo(code)
            except (KeyError, OSError):
                return None

        absx = absinfo(ec.ABS_X)
        # Pro Controller right stick vertical = ABS_RY; Nimbus (BLE HID) = ABS_RZ
        thr_code, absthr = ec.ABS_RY, absinfo(ec.ABS_RY)
        if absthr is None or absthr.max == absthr.min:
            alt = absinfo(ec.ABS_RZ)
            if alt is not None:
                thr_code, absthr = ec.ABS_RZ, alt
        st.log(f'gamepad: steer=ABS_X throttle={"ABS_RY" if thr_code == ec.ABS_RY else "ABS_RZ"}')

        # Adaptive per-axis range: worn sticks (this pad's right stick reaches
        # ~29% of spec) get normalized against the range they ACTUALLY produce.
        learned = {}

        def norm(v, info, code):
            if info is None:
                return 0.0
            center = (info.max + info.min) / 2.0
            dev_ = v - center
            ext = learned.get(code, 4000.0)
            if abs(dev_) > ext:
                ext = learned[code] = abs(dev_)
            n = dev_ / ext
            if abs(n) < DEADZONE:
                return 0.0
            # re-span so full learned deflection = 1.0 past the deadzone
            n = (abs(n) - DEADZONE) / (1.0 - DEADZONE) * (1 if n > 0 else -1)
            return max(-1.0, min(1.0, n))

        ESTOP_BTNS = (ec.BTN_SOUTH, ec.BTN_EAST)      # B on Nintendo, A/B alias
        MODE_BTNS = (ec.BTN_START, ec.BTN_MODE)       # PLUS / Menu

        steer, thr = 0.0, 0.0
        last_ev = time.monotonic()
        while st.running:
            r, _, _ = select([dev.fd], [], [], 0.05)
            now = time.monotonic()
            if r:
                for ev in dev.read():
                    if ev.type == ec.EV_ABS:
                        if ev.code == ec.ABS_X:
                            steer = norm(ev.value, absx, ev.code)
                            if st.config.get("gp_invert_steer", True):
                                steer = -steer
                        elif ev.code == thr_code:
                            thr = -norm(ev.value, absthr, ev.code)  # up = fwd
                    elif ev.type == ec.EV_KEY and ev.value == 1:
                        if ev.code in ESTOP_BTNS:
                            st.estop = not st.estop
                            st.log(f'gamepad: estop '
                                   f'{"ON" if st.estop else "OFF"}')
                        elif ev.code in MODE_BTNS:
                            if st.mode in (P.MODE_MANUAL, P.MODE_ASSIST,
                                           P.MODE_AUTO):
                                st.set_mode((st.mode + 1) % 3)
                            else:
                                st.set_mode(P.MODE_MANUAL)
                last_ev = now
                st.user_steer = steer
                st.user_throttle = thr
                st.last_input_time = now
                st.last_input_src = 'gamepad'
            elif now - last_ev > DEADMAN_S:
                self._neutral_if_owner()
