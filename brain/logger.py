"""CSV logger @10Hz -> /home/pi/cc02/logs/log_YYYYmmdd_HHMMSS.csv"""
import math
import os
import threading
import time

LOG_DIR = '/home/pi/cc02/logs'
HEADER = ('timestamp,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,'
          'servo_us_in,esc_us_in,steer_us_out,throttle_us_out,'
          'mode,det_count,collision\n')


class CsvLogger(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True, name='logger')
        self.state = state
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(
            LOG_DIR, time.strftime('log_%Y%m%d_%H%M%S.csv'))
        self._fh = open(self.path, 'w')
        self._fh.write(HEADER)

    @staticmethod
    def latest():
        try:
            files = sorted(f for f in os.listdir(LOG_DIR)
                           if f.startswith('log_') and f.endswith('.csv'))
            return os.path.join(LOG_DIR, files[-1]) if files else None
        except OSError:
            return None

    def run(self):
        st = self.state
        n = 0
        while st.running:
            t = st.telem or {}
            row = (
                f"{time.time():.2f},"
                f"{t.get('ax', 0):.3f},{t.get('ay', 0):.3f},{t.get('az', 0):.3f},"
                f"{t.get('gx', 0):.3f},{t.get('gy', 0):.3f},{t.get('gz', 0):.3f},"
                f"{math.degrees(t.get('roll', 0)):.1f},"
                f"{math.degrees(t.get('pitch', 0)):.1f},"
                f"{t.get('servo_us', 0)},{t.get('esc_us', 0)},"
                f"{st.out_steer_us},{st.out_throttle_us},"
                f"{st.mode_name()},{st.det_count},{int(st.collision)}\n")
            try:
                self._fh.write(row)
                n += 1
                if n % 10 == 0:
                    self._fh.flush()
            except Exception:
                pass
            time.sleep(0.1)
