"""Breadcrumb + detection map log. Dead-reckoning, BEST-EFFORT - not SLAM.
Appends jsonl to /home/pi/cc02/logs/map.jsonl; keeps last 2000 points in memory.
"""
import json
import os
import threading
import time
from collections import deque

LOG_PATH = '/home/pi/cc02/logs/map.jsonl'


class Mapping:
    def __init__(self, state):
        self.state = state
        self.crumbs = deque(maxlen=2000)
        self.dets = deque(maxlen=500)
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        self._fh = open(LOG_PATH, 'a', buffering=1)

    def add_crumb(self, x, y, heading):
        rec = {'t': round(time.time(), 2), 'type': 'crumb',
               'x': round(x, 3), 'y': round(y, 3), 'heading': round(heading, 3)}
        with self._lock:
            self.crumbs.append(rec)
            self._fh.write(json.dumps(rec) + '\n')

    def add_detection(self, cls, conf, x, y, heading):
        rec = {'t': round(time.time(), 2), 'type': 'det', 'cls': cls,
               'conf': round(conf, 2), 'x': round(x, 3), 'y': round(y, 3),
               'heading': round(heading, 3)}
        with self._lock:
            self.dets.append(rec)
            self._fh.write(json.dumps(rec) + '\n')

    def snapshot(self):
        with self._lock:
            return {'crumbs': list(self.crumbs), 'dets': list(self.dets)}
