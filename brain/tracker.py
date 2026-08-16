"""SORT-style multi-object tracker. NumPy only, no extra deps.

IDs are stable across frames so AUTO can lock a person track instead of
re-picking the biggest box every tick. Velocities are image-plane px/s
(EMA); ground-frame speed is filled in later by occupancy.project_tracks.
"""
from __future__ import annotations


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    ua = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    ub = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    den = ua + ub - inter
    return inter / den if den > 1e-6 else 0.0


class Track:
    __slots__ = (
        'tid', 'cls', 'x1', 'y1', 'x2', 'y2', 'conf',
        'vx', 'vy', 'hits', 'age', 'missed',
        'gx', 'gy', 'gvx', 'gvy',
    )

    def __init__(self, tid, det):
        x1, y1, x2, y2, cls, conf = det
        self.tid = int(tid)
        self.cls = cls
        self.x1, self.y1, self.x2, self.y2 = float(x1), float(y1), float(x2), float(y2)
        self.conf = float(conf)
        self.vx = 0.0
        self.vy = 0.0
        self.hits = 1
        self.age = 1
        self.missed = 0
        self.gx = None   # body-frame ground, filled by occupancy.project_track
        self.gy = None
        self.gvx = 0.0
        self.gvy = 0.0

    @property
    def bbox(self):
        return (self.x1, self.y1, self.x2, self.y2)

    @property
    def cx(self):
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self):
        return 0.5 * (self.y1 + self.y2)

    def as_tuple(self):
        return (self.tid, self.cls, self.x1, self.y1, self.x2, self.y2,
                self.conf, self.vx, self.vy, self.gx, self.gy)

    def to_dict(self):
        return {
            'id': self.tid, 'cls': self.cls, 'conf': round(self.conf, 2),
            'x1': int(self.x1), 'y1': int(self.y1),
            'x2': int(self.x2), 'y2': int(self.y2),
            'gx': None if self.gx is None else round(self.gx, 2),
            'gy': None if self.gy is None else round(self.gy, 2),
        }


class SortTracker:
    """Greedy IoU tracker. Prefer same-class matches. Confirm after 2 hits."""

    def __init__(self, iou_thr=0.3, max_missed=8, min_hits=2):
        self.iou_thr = float(iou_thr)
        self.max_missed = int(max_missed)
        self.min_hits = int(min_hits)
        self._next_id = 1
        self.tracks = []

    def update(self, dets, dt=0.05):
        """dets: iterable of (x1,y1,x2,y2,cls,conf). Returns confirmed tracks."""
        dt = max(1e-3, float(dt))
        dets = list(dets)
        n_t, n_d = len(self.tracks), len(dets)
        used_t = [False] * n_t
        used_d = [False] * n_d
        pairs = []
        for i, tr in enumerate(self.tracks):
            for j, det in enumerate(dets):
                iou = _iou(tr.bbox, det[:4])
                if det[4] != tr.cls:
                    iou *= 0.35
                if iou >= self.iou_thr:
                    pairs.append((iou, i, j))
        pairs.sort(reverse=True)
        for iou, i, j in pairs:
            if used_t[i] or used_d[j]:
                continue
            used_t[i] = used_d[j] = True
            self._apply(self.tracks[i], dets[j], dt)

        for i, tr in enumerate(self.tracks):
            if not used_t[i]:
                tr.missed += 1
                tr.age += 1

        for j, det in enumerate(dets):
            if not used_d[j]:
                self.tracks.append(Track(self._next_id, det))
                self._next_id += 1

        self.tracks = [tr for tr in self.tracks if tr.missed <= self.max_missed]
        return [tr for tr in self.tracks
                if tr.hits >= self.min_hits and tr.missed == 0]

    def get(self, tid):
        for tr in self.tracks:
            if tr.tid == tid:
                return tr
        return None

    def _apply(self, tr, det, dt):
        x1, y1, x2, y2, cls, conf = det
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        vx = (cx - tr.cx) / dt
        vy = (cy - tr.cy) / dt
        tr.vx = 0.6 * tr.vx + 0.4 * vx
        tr.vy = 0.6 * tr.vy + 0.4 * vy
        tr.x1, tr.y1, tr.x2, tr.y2 = float(x1), float(y1), float(x2), float(y2)
        tr.cls = cls
        tr.conf = float(conf)
        tr.hits += 1
        tr.age += 1
        tr.missed = 0
