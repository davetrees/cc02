"""Vision thread: YOLO on Hailo-10H (primary) with CPU fallback + wall heuristic.

Backends:
- hailo: yolov8m_h10.hef on the Hailo-10H via hailo_platform InferVStreams.
  Input 640x640 RGB uint8 (letterboxed); NMS runs on-device, output is
  per-class [ymin,xmin,ymax,xmax,score] normalized to the letterboxed input.
- cpu-fallback: ultralytics yolov8n, imgsz=320, thread-capped (undervoltage).
Any Hailo failure (import, device absent/busy, configure, repeated runtime
errors) drops to the CPU path automatically - the car never loses vision.

Heuristic (COCO has no wall class): grayscale center band (middle 1/3 columns,
lower half rows), Canny edges -> lowest edge row as range proxy, plus Laplacian
variance. Low texture + no floor edges ahead => wall-like obstacle.
Collision also when any enabled-class YOLO box area fraction > threshold and
box center within the middle 1/2 of the frame.

Outputs for the AUTO cruiser:
- st.col_open: 9 columns, each normalized 0..1 (1 = floor edge visible close
  to the car = open; 0 = no near floor edge = blocked).
- st.boxes: class-filtered detections (collision + annotation, as before).
- st.boxes_all: ALL detections above conf, class filter ignored (cost map).
- st.frame_w/frame_h, st.vision_time (staleness guard).
"""
import contextlib
import threading
import time

import cv2
import numpy as np

cv2.setNumThreads(2)  # undervoltage mitigation: cap CPU load

HEF_PATH = '/usr/share/hailo-models/yolov8m_h10.hef'

COCO80 = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush']


class HailoBackend:
    """yolov8m_h10.hef via the hailo_platform InferModel (async) API.

    Probe-verified on this Hailo-10H (2026-08-16): the legacy InferVStreams
    path returns HAILO_NOT_IMPLEMENTED on H10; create_infer_model works.
    Input (640,640,3) uint8, no batch dim. Output buffer flat float32
    (40080,) = 80 classes x (1 + 100*5); get_buffer() returns the parsed
    list of 80 per-class (n,5) [ymin,xmin,ymax,xmax,score] arrays.
    """

    def __init__(self, log):
        from hailo_platform import VDevice, HailoSchedulingAlgorithm
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self.vdev = VDevice(params)
        self.im = self.vdev.create_infer_model(HEF_PATH)
        self.im.set_batch_size(1)
        in_shape = tuple(self.im.input().shape)   # (640, 640, 3)
        self.in_h, self.in_w = in_shape[0], in_shape[1]
        self._stack = contextlib.ExitStack()
        self.cim = self._stack.enter_context(self.im.configure())
        self.bindings = self.cim.create_bindings()
        self.out_buf = np.empty(tuple(self.im.output().shape), dtype=np.float32)
        self.bindings.output().set_buffer(self.out_buf)
        # validate end-to-end with a dummy frame before claiming the backend
        self.bindings.input().set_buffer(
            np.zeros((self.in_h, self.in_w, 3), dtype=np.uint8))
        self.cim.run([self.bindings], 5000)
        log('hailo: InferModel pipeline validated (dummy inference ok)')

    def infer(self, frame_bgr):
        """Returns [(x1,y1,x2,y2,cls_name,score)] in camera-frame pixel coords."""
        h, w = frame_bgr.shape[:2]
        scale = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        if (nw, nh) != (w, h):
            rgb = cv2.resize(rgb, (nw, nh))
        pad_x = (self.in_w - nw) // 2
        pad_y = (self.in_h - nh) // 2
        canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = rgb
        self.bindings.input().set_buffer(canvas)
        self.cim.run([self.bindings], 2000)
        per_class = self.bindings.output().get_buffer()  # 80 x (n,5) arrays
        dets = []
        for cls_id, arr in enumerate(per_class):
            a = np.asarray(arr)
            if a.size == 0:
                continue
            if a.ndim == 2 and a.shape[0] == 5 and a.shape[1] != 5:
                a = a.T  # (5, n) -> (n, 5)
            a = a.reshape(-1, 5)
            for ymin, xmin, ymax, xmax, score in a:
                if score <= 0.0:
                    continue
                x1 = (xmin * self.in_w - pad_x) / scale
                y1 = (ymin * self.in_h - pad_y) / scale
                x2 = (xmax * self.in_w - pad_x) / scale
                y2 = (ymax * self.in_h - pad_y) / scale
                dets.append((int(max(0, min(w, x1))), int(max(0, min(h, y1))),
                             int(max(0, min(w, x2))), int(max(0, min(h, y2))),
                             COCO80[cls_id] if cls_id < 80 else str(cls_id),
                             float(score)))
        return dets

    def close(self):
        try:
            self._stack.close()
        except Exception:
            pass


class Vision(threading.Thread):
    def __init__(self, state, camera, mapping=None):
        super().__init__(daemon=True, name='vision')
        self.state = state
        self.camera = camera
        self.mapping = mapping
        self._floor_hist = None
        self.backend = 'none'
        self.hailo = None
        self.model = None          # CPU ultralytics model
        self.names = {}            # CPU model names
        self._fps_ema = 0.0
        self._last_det_log = 0.0
        self._first_logged = False
        self._hailo_fails = 0
        self._last_frame_no = -1
        # hailo boot-race handling: the H10 answers HAILO_CONNECTION_REFUSED
        # for a while after boot. Retry every 10s (12 tries) before CPU
        # fallback; once on CPU, still probe every 120s to upgrade back.
        self._hailo_attempts = 0
        self._next_hailo_try = 0.0

    # ---------- backend init ----------
    def _init_hailo(self):
        st = self.state
        try:
            self.hailo = HailoBackend(st.log)
            self.backend = 'hailo'
            st.log('vision: backend=hailo (yolov8m_h10.hef, 640x640, NMS on-device)')
            return True
        except Exception as e:
            st.log(f'vision: hailo init failed ({type(e).__name__}: {e})')
            self.hailo = None
            return False

    def _init_cpu(self):
        st = self.state
        try:
            import torch
            torch.set_num_threads(2)  # undervoltage mitigation: cap CPU load
            from ultralytics import YOLO
            self.model = YOLO('/home/pi/cc02/yolov8n.pt')
            self.names = self.model.names
            self.backend = 'cpu-fallback'
            st.log('vision: backend=cpu-fallback (yolov8n, imgsz=320, 2 threads)')
            return True
        except Exception as e:
            st.log(f'vision: CPU backend load FAILED ({e}); heuristic-only mode')
            self.model = None
            self.backend = 'heuristic-only'
            return False

    def _hailo_retry_tick(self, now):
        """Non-blocking Hailo acquisition/upgrade; heuristic keeps running."""
        st = self.state
        if self.backend == 'hailo' or now < self._next_hailo_try:
            return
        if self.backend == 'none':
            self._hailo_attempts += 1
            if self._init_hailo():
                return
            if self._hailo_attempts >= 12:
                st.log('vision: hailo not ready after 12 tries, falling back to CPU '
                       '(will keep probing every 120s)')
                self._init_cpu()
                self._next_hailo_try = now + 120.0
            else:
                self._next_hailo_try = now + 10.0
        else:  # cpu-fallback / heuristic-only: background upgrade probe
            if self._init_hailo():
                st.log('vision: upgraded back to hailo backend')
                self.model = None  # release CPU model memory
            else:
                self._next_hailo_try = now + 120.0

    def run(self):
        st = self.state
        st.log('vision: starting, trying Hailo backend first')

        while st.running:
            self._hailo_retry_tick(time.monotonic())
            frame = self.camera.get_frame()
            if frame is None:
                time.sleep(0.2)
                continue
            # process each camera frame once (camera-capped, saves power)
            fno = self.camera.frame_count
            if fno == self._last_frame_no:
                time.sleep(0.005)
                continue
            self._last_frame_no = fno

            t0 = time.monotonic()
            try:
                self._heuristic(frame)
            except Exception as e:
                st.log(f'vision: heuristic error {e}')

            boxes, all_boxes = [], []
            ran_inference = False
            if st.config.get('yolo_enable', True):
                if self.backend == 'hailo':
                    boxes, all_boxes, ran_inference = self._run_hailo(frame)
                elif self.backend == 'cpu-fallback':
                    try:
                        boxes, all_boxes = self._yolo_cpu(frame)
                        ran_inference = True
                    except Exception as e:
                        st.log(f'vision: yolo error {e}')
            if ran_inference:
                dt = time.monotonic() - t0
                if dt > 0:
                    self._fps_ema = 0.7 * self._fps_ema + 0.3 * (1.0 / dt)
                st.yolo_fps = self._fps_ema
                if not self._first_logged:
                    self._first_logged = True
                    st.log(f'vision: first inference done backend={self.backend} '
                           f'({1.0 / max(dt, 1e-6):.1f} fps, {len(boxes)} dets)')
            st.boxes = boxes
            st.boxes_all = all_boxes
            st.det_count = len(boxes)
            self._collision(frame, boxes)
            self._log_dets(boxes)
            if not ran_inference:
                time.sleep(0.1)  # heuristic-only: ~10Hz

    # ---------- hailo path ----------
    def _run_hailo(self, frame):
        st = self.state
        try:
            dets = self.hailo.infer(frame)
            self._hailo_fails = 0
        except Exception as e:
            self._hailo_fails += 1
            st.log(f'vision: hailo runtime error {self._hailo_fails}/5 ({e})')
            if self._hailo_fails >= 5:
                st.log('vision: hailo failed repeatedly, switching to CPU fallback')
                if self.hailo:
                    self.hailo.close()
                    self.hailo = None
                self._init_cpu()
                self._next_hailo_try = time.monotonic() + 120.0
            return [], [], False
        conf = float(st.config.get('yolo_conf', 0.35))
        allowed = set(st.config.get('yolo_classes', []))
        # contract: (x1,y1,x2,y2,cls_name,conf)
        all_boxes = [d for d in dets if d[5] >= conf]
        boxes = [d for d in all_boxes if not allowed or d[4] in allowed]
        return boxes, all_boxes, True

    # ---------- cpu path ----------
    def _yolo_cpu(self, frame):
        st = self.state
        conf = float(st.config.get('yolo_conf', 0.35))
        allowed = set(st.config.get('yolo_classes', []))
        res = self.model.predict(frame, imgsz=320, conf=conf, verbose=False)[0]
        boxes, all_boxes = [], []
        for b in res.boxes:
            cls = self.names.get(int(b.cls[0]), str(int(b.cls[0])))
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0]]
            det = (x1, y1, x2, y2, cls, float(b.conf[0]))
            all_boxes.append(det)
            if not allowed or cls in allowed:
                boxes.append(det)
        return boxes, all_boxes

    # ---------- detection -> map log (throttled) ----------
    def _log_dets(self, boxes):
        st = self.state
        now = time.monotonic()
        if boxes and self.mapping and now - self._last_det_log > 0.5:
            self._last_det_log = now
            for (x1, y1, x2, y2, cls, c) in boxes:
                self.mapping.add_detection(cls, c, st.pose_x, st.pose_y, st.pose_h)

    # ---------- heuristic ----------
    def _heuristic(self, frame):
        # Floor-model runway sensor (field-tuned for backyard 2026-08-16):
        # backproject a lawn color model sampled from the strip right in front,
        # gate by isotropic texture (kills corrugated fence + smooth walls +
        # sky), cap at the horizon, then per-column contiguous runway height.
        st = self.state
        small = cv2.resize(frame, (320, 240))
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        h, w = 240, 320
        hist = cv2.calcHist([hsv[int(h*0.85):, int(w*0.2):int(w*0.8)]],
                            [0, 1], None, [30, 32], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
        if self._floor_hist is None:
            self._floor_hist = hist
        elif not st.collision:
            # slow EMA; frozen while blocked so a wall in the sample strip
            # cannot become "floor"
            self._floor_hist = 0.9 * self._floor_hist + 0.1 * hist
        bp = cv2.calcBackProject([hsv], [0, 1],
                                 self._floor_hist.astype(np.float32),
                                 [0, 180, 0, 256], 1)
        bp = cv2.GaussianBlur(bp, (9, 9), 0)
        _, cmask = cv2.threshold(bp, 40, 255, cv2.THRESH_BINARY)
        ax = cv2.boxFilter(np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)), -1, (15, 15))
        ay = cv2.boxFilter(np.abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)), -1, (15, 15))
        tmask = ((ay > 10) & (ax < 2.5 * ay + 20)).astype(np.uint8) * 255
        mask = cv2.bitwise_and(cmask, tmask)
        mask[: int(h * 0.45)] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        runway = []
        for c in range(9):
            x0, x1 = int(c * w / 9), int((c + 1) * w / 9)
            col = mask[:, x0:x1].mean(axis=1) > 153
            r, gap = 0, 0
            for y in range(h - 1, int(h * 0.4), -1):
                if col[y]:
                    r += 1; gap = 0
                else:
                    gap += 1
                    if gap > 5:
                        break
                    r += 1
            runway.append(round(max(0, r - gap) / (h * 0.55), 3))
        st.col_open = runway
        _cf = float(st.config.get("path_center_frac", 0.34))
        _cols = [i for i in range(9) if (i+1)/9.0 > 0.5-_cf/2 and i/9.0 < 0.5+_cf/2] or [4]
        cmin = min(runway[i] for i in _cols)
        st.wall_like = bool(cmin < float(st.config.get("runway_block", 0.18)))
        st.range_proxy = cmin
        st.lap_var = 0.0
        st.frame_w, st.frame_h = frame.shape[1], frame.shape[0]
        st.vision_time = time.monotonic()

    # ---------- collision ----------
    def _collision(self, frame, boxes):
        st = self.state
        h, w = frame.shape[:2]
        area = float(w * h)
        thr = float(st.config.get('collision_area_threshold', 0.20))
        # car-path region of interest (fractions of frame): only obstacles that
        # overlap this rectangle count. Tune live to match the car's real path.
        cf = float(st.config.get('path_center_frac', 0.34))   # width
        tf = float(st.config.get('path_top_frac', 0.35))      # ignore above (distant/sky)
        bf = float(st.config.get('path_bottom_frac', 0.05))   # ignore very bottom
        rx0 = w * (0.5 - cf / 2.0); rx1 = w * (0.5 + cf / 2.0)
        ry0 = h * tf;               ry1 = h * (1.0 - bf)
        yolo_close = False
        for (x1, y1, x2, y2, cls, c) in boxes:
            frac = ((x2 - x1) * (y2 - y1)) / area
            # box must OVERLAP the ROI rectangle in BOTH axes
            if frac > thr and x1 < rx1 and x2 > rx0 and y1 < ry1 and y2 > ry0:
                yolo_close = True
                break
        st.collision = bool(yolo_close or st.wall_like)

    # ---------- annotation for MJPEG stream ----------
    def annotate(self, frame):
        st = self.state
        # Draw EVERY detected object (identification is decoupled from the
        # collision class filter). Collision-armed classes (in yolo_classes)
        # are flagged orange; info-only detections green.
        allowed = set(st.config.get('yolo_classes', []))
        for (x1, y1, x2, y2, cls, c) in list(st.boxes_all):
            armed = (not allowed) or (cls in allowed)
            color = (0, 165, 255) if armed else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3 if armed else 2)
            cv2.putText(frame, f'{cls} {c:.2f}', (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        fw = frame.shape[1]; fh = frame.shape[0]
        _cf = float(st.config.get('path_center_frac', 0.34))
        _tf = float(st.config.get('path_top_frac', 0.35))
        _bf = float(st.config.get('path_bottom_frac', 0.05))
        _x0 = int(fw * (0.5 - _cf / 2)); _x1 = int(fw * (0.5 + _cf / 2))
        _y0 = int(fh * _tf); _y1 = int(fh * (1.0 - _bf))
        cv2.rectangle(frame, (_x0, _y0), (_x1, _y1), (0, 200, 255), 2)
        cv2.putText(frame, 'path', (_x0 + 3, _y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if st.collision:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 32), (0, 0, 255), -1)
            cv2.putText(frame, 'COLLISION', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return frame
