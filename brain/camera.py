"""Single camera capture thread with latest-frame buffer. Opens /dev/video0 exactly once."""
import threading
import time

import cv2


class Camera(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True, name='camera')
        self.state = state
        self._lock = threading.Lock()
        self._frame = None
        self.frame_count = 0
        self.opened = False

    def _open(self):
        """Open /dev/video0 with MJPG 640x480@30; props re-applied every open."""
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cw = int(self.state.config.get('cam_width', 1920))
        ch = int(self.state.config.get('cam_height', 1080))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ch)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.state.log(f'camera: opened /dev/video0 {int(w)}x{int(h)} MJPG')
        return cap

    def run(self):
        st = self.state
        cap = self._open()
        if cap is None:
            st.log('camera: FAILED to open /dev/video0 (will retry every 2s)')
        else:
            self.opened = True
        fails = 0
        while st.running:
            if cap is None:
                time.sleep(2.0)
                cap = self._open()
                if cap is not None:
                    self.opened = True
                    fails = 0
                continue
            ok, frame = cap.read()
            if ok and frame is not None:
                with self._lock:
                    self._frame = frame
                self.frame_count += 1
                fails = 0
            else:
                fails += 1
                if fails == 30:
                    st.log('camera: repeated read failures, reopening device')
                    cap.release()
                    cap = None
                    continue
                time.sleep(0.05)
        if cap is not None:
            cap.release()

    def get_frame(self):
        with self._lock:
            if self._frame is None:
                return None
            return self._frame.copy()
