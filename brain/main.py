"""CC-02 brain: wires camera, vision, serial link, autopilot, logger, web panel."""
import json
import os
import sys
import time

from aiohttp import web as aioweb

import protocol as P

CONFIG_PATH = '/home/pi/cc02/config.json'
DEFAULT_CONFIG = {
    'yolo_enable': True,
    'yolo_conf': 0.35,
    'yolo_classes': ['person', 'car', 'chair', 'dog', 'cat', 'bottle', 'backpack'],
    'collision_stop': True,
    'collision_area_threshold': 0.20,
    'max_speed': 0.30,          # fraction of 500us; panel edits it in us
    'anti_tip_enable': True,
    'tip_roll_deg': 45,
    'tip_pitch_deg': 45,
    # AUTO cruiser (autopilot.py)
    'auto_cruise_us': 120,      # forward deviation at full clearance
    'auto_hyst': 0.15,          # cost delta required to move the steer target
    'auto_open_thr': 0.65,      # center clearance >= this -> full cruise
    'auto_block_thr': 0.25,     # center clearance <= this -> stop
    'auto_floor_weight': 1.0,   # Canny openness -> cost weight
    'auto_yolo_gain': 3.0,      # YOLO box area fraction -> cost gain
    'auto_person_margin': 2.0,  # person box width safety multiplier
    'auto_person_area': 0.02,   # min person area fraction to count as obstruction
    'auto_block_time_s': 0.7,   # blocked/collision this long -> escape
    'auto_stop_time_s': 0.3,    # escape: neutral pause before reversing
    'auto_reverse_time_s': 0.8, # escape: reverse duration
    'auto_reverse_us': 140,     # escape: reverse deviation (capped by max_speed)
    'auto_escape_steer_us': 300,# escape: steer deviation away from obstruction
    'stuck_time_s': 2.0,        # forward commanded this long with no vibration
    'stuck_accel_var': 0.02,    # sum of xyz accel variances below this = stuck
    'override_thr': 0.3,        # |user input| above this in AUTO/RTH -> MANUAL
    # web-panel-only input inversion (see web.py ws handler)
    'web_invert_steer': True,
    'web_invert_throttle': True,
    # Pi-paired gamepad steer inversion (gamepad.py)
    'gp_invert_steer': True,
    # anti-tip counter-steer gate (autopilot.py; needs a zeroed AH to be safe)
    'counter_steer_enable': False,
}


class State:
    def __init__(self):
        self.running = True
        self.config = dict(DEFAULT_CONFIG)
        self.load_config()
        # control
        self.mode = P.MODE_MANUAL
        self.estop = False
        self.user_steer = 0.0
        self.user_throttle = 0.0
        self.last_input_time = 0.0
        self.out_steer_us = 1500
        self.out_throttle_us = 1500
        # serial
        self.telem = {}
        self.last_telem_time = 0.0
        self.telem_hz = 0.0
        self.link_connected = False
        # vision
        self.boxes = []            # class-filtered dets (collision/annotation)
        self.boxes_all = []        # ALL dets above conf (AUTO cost map)
        self.det_count = 0
        self.yolo_fps = 0.0
        self.collision = False
        self.wall_like = False
        self.range_proxy = -1
        self.lap_var = 0.0
        self.col_open = [0.0] * 9  # 9 columns, 0..1 openness
        self.frame_w = 640
        self.frame_h = 480
        self.vision_time = 0.0     # monotonic time of last heuristic update
        # AUTO cruiser observability (published by autopilot every tick)
        self.auto_costs = [1.0] * 9
        self.auto_target = 4
        self.auto_steer_us = 1500
        self.auto_state = 'CRUISE'
        self.auto_accel_var = None
        # pose (dead reckoning, best-effort)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_h = 0.0
        self.rth_remaining = 0
        # web
        self.ws_clients = 0
        # LEVEL/ZERO: panel sets this timestamp; serial_link raises the DISP
        # CAL_REQUEST flag (bit1) while it is <1s old so the ESP re-zeros AH
        self.cal_request_time = 0.0
        # direct-paired Pro Controller
        self.gamepad_connected = False
        self.last_input_src = ''

    def log(self, msg):
        print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)

    def set_mode(self, mode):
        self.mode = mode
        self.log(f'mode -> {P.MODE_NAMES.get(mode, mode)}')

    def mode_name(self):
        return 'ESTOP' if self.estop else P.MODE_NAMES.get(self.mode, '?')

    def load_config(self):
        try:
            with open(CONFIG_PATH) as f:
                self.config.update(json.load(f))
        except (OSError, ValueError):
            pass

    def save_config(self):
        try:
            with open(CONFIG_PATH, 'w') as f:
                json.dump(self.config, f, indent=1)
        except OSError as e:
            self.log(f'config save failed: {e}')

    def cmd_tuple(self):
        """(steer_us, throttle_us, mode, flags, tip_roll, tip_pitch) for CMD frame."""
        flags = P.CF_FAILSAFE_STEER_CENTER
        if self.estop:
            flags |= P.CF_ESTOP
        if self.config.get('anti_tip_enable', True):
            flags |= P.CF_TIP_ENABLE
        mode = P.MODE_ESTOP if self.estop else self.mode
        steer = 1500 if self.estop else self.out_steer_us
        thr = 1500 if self.estop else self.out_throttle_us
        # servo/ESC direction inversion (config; default inverted per install)
        if self.config.get("invert_steer", True):
            steer = 3000 - steer
        if self.config.get("invert_throttle", True):
            thr = 3000 - thr
        return (steer, thr, mode, flags,
                self.config.get('tip_roll_deg', 45),
                self.config.get('tip_pitch_deg', 45))


def main():
    os.makedirs('/home/pi/cc02/logs', exist_ok=True)
    state = State()
    state.log('cc02 brain starting')

    from camera import Camera
    from mapping import Mapping
    from serial_link import SerialLink
    from logger import CsvLogger

    camera = Camera(state)
    mapping = Mapping(state)

    from vision import Vision
    vision = Vision(state, camera, mapping)

    from autopilot import Autopilot
    autopilot = Autopilot(state, mapping)

    serial_link = SerialLink(state)
    csvlog = CsvLogger(state)

    from gamepad import Gamepad
    gamepad = Gamepad(state)

    camera.start()
    serial_link.start()
    autopilot.start()
    csvlog.start()
    vision.start()
    gamepad.start()

    import web as webmod
    app = webmod.make_app(state, camera, vision, mapping)

    # localhost-only debug/exec port (127.0.0.1:8079, SSH access only)
    import debugport

    async def _start_debugport(_app):
        await debugport.start(state, extra={
            'P': P, 'protocol': P, 'camera': camera, 'vision': vision,
            'mapping': mapping, 'autopilot': autopilot,
            'serial_link': serial_link, 'gamepad': gamepad, 'csvlog': csvlog,
            'time': time, 'json': json})

    app.on_startup.append(_start_debugport)
    state.log('web: starting on 0.0.0.0:8080')
    try:
        aioweb.run_app(app, host='0.0.0.0', port=8080,
                       access_log=None, handle_signals=True)
    finally:
        state.running = False
        state.log('shutting down threads...')
        # join C++-backed threads before interpreter exit; abrupt teardown of
        # cv2/hailo/pyserial threads aborts the process (status=6/ABRT)
        for t in (vision, autopilot, csvlog, serial_link, camera, gamepad):
            t.join(timeout=3)
        try:
            if getattr(vision, 'hailo', None):
                vision.hailo.close()
        except Exception:
            pass
        state.log('cc02 brain stopped')


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    main()
