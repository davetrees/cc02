"""Egocentric occupancy grid. Body frame: x forward, y left, origin at chassis.

Built each vision frame from (1) the floor-runway prior, (2) tracked YOLO
boxes projected through a ground-plane homography. Nothing here is stored
in world frame — dead-reckoned pose is not an obstacle source of truth.
"""
from __future__ import annotations

import math

import numpy as np

OCC_FREE = 0.12
OCC_UNK = 0.50
OCC_BLOCK = 0.92
OCC_THR = 0.70

SRC_UNK = 0
SRC_FREE = 1
SRC_RUNWAY = 2   # floor heuristic says blocked
SRC_YOLO = 3

CLASS_R = {
    'person': 0.32, 'chair': 0.30, 'couch': 0.55, 'bed': 0.55,
    'potted plant': 0.28, 'dining table': 0.48, 'tv': 0.30,
    'car': 0.80, 'bicycle': 0.40, 'motorcycle': 0.45,
    'bottle': 0.12, 'backpack': 0.22, 'suitcase': 0.28,
    'dog': 0.28, 'cat': 0.20, 'bench': 0.40,
}


def cfg_f(cfg, key, default):
    try:
        return float(cfg.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def cfg_i(cfg, key, default):
    try:
        return int(cfg.get(key, default))
    except (TypeError, ValueError):
        return int(default)


def camera_intrinsics(cfg, frame_w, frame_h):
    fx = cfg_f(cfg, 'cam_fx', 520.0)
    fy = cfg_f(cfg, 'cam_fy', 520.0)
    cx = cfg_f(cfg, 'cam_cx', frame_w * 0.5)
    cy = cfg_f(cfg, 'cam_cy', frame_h * 0.5)
    return fx, fy, cx, cy


def pixel_to_ground(u, v, cfg, frame_w=640, frame_h=480):
    """Pinhole + pitch-down camera -> ground (x forward, y left) or None.

    Camera optical: x right, y down, z forward. Tilted down by cam_tilt_deg.
    Uncalibrated defaults are honest approximations, not a surveyed extrinsics.
    """
    h = cfg_f(cfg, 'cam_height_m', 0.16)
    tilt = math.radians(cfg_f(cfg, 'cam_tilt_deg', 22.0))
    cam_x = cfg_f(cfg, 'cam_x_m', 0.08)
    cam_y = cfg_f(cfg, 'cam_y_m', 0.0)
    fx, fy, cx, cy = camera_intrinsics(cfg, frame_w, frame_h)
    xn = (float(u) - cx) / max(1e-6, fx)
    yn = (float(v) - cy) / max(1e-6, fy)
    # Z_opt * (cos(tilt)*yn + sin(tilt)) = h   (see module docstring in git)
    den = math.cos(tilt) * yn + math.sin(tilt)
    if den <= 1e-4:
        return None
    z_opt = h / den
    if z_opt <= 0.05 or z_opt > 12.0:
        return None
    x = z_opt * (math.sin(tilt) * yn + math.cos(tilt)) + cam_x
    y = -z_opt * xn + cam_y
    if x < -0.8 or x > 8.0 or abs(y) > 4.0:
        return None
    return x, y


def pixel_to_ground_arr(u, v, cfg, frame_w, frame_h):
    """Vectorized pixel_to_ground. u,v arrays -> x,y arrays with nan invalid."""
    h = cfg_f(cfg, 'cam_height_m', 0.16)
    tilt = math.radians(cfg_f(cfg, 'cam_tilt_deg', 22.0))
    cam_x = cfg_f(cfg, 'cam_x_m', 0.08)
    cam_y = cfg_f(cfg, 'cam_y_m', 0.0)
    fx, fy, cx, cy = camera_intrinsics(cfg, frame_w, frame_h)
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    xn = (u - cx) / max(1e-6, fx)
    yn = (v - cy) / max(1e-6, fy)
    den = math.cos(tilt) * yn + math.sin(tilt)
    z_opt = np.full(u.shape, np.nan)
    ok = den > 1e-4
    z_opt[ok] = h / den[ok]
    ok &= (z_opt > 0.05) & (z_opt < 12.0)
    x = z_opt * (math.sin(tilt) * yn + math.cos(tilt)) + cam_x
    y = -z_opt * xn + cam_y
    ok &= (x >= -0.8) & (x <= 8.0) & (np.abs(y) <= 4.0)
    x = np.where(ok, x, np.nan)
    y = np.where(ok, y, np.nan)
    return x, y


class Grid:
    """rows along +x (forward), cols along +y (left). Cell (r,c) center in metres."""

    def __init__(self, cols=24, rows=40, res=0.12, origin_x=-0.48, origin_y=None):
        self.cols = int(cols)
        self.rows = int(rows)
        self.res = float(res)
        self.origin_x = float(origin_x)
        if origin_y is None:
            origin_y = -0.5 * self.cols * self.res
        self.origin_y = float(origin_y)
        self.occ = np.full((self.rows, self.cols), OCC_UNK, dtype=np.float32)
        self.src = np.zeros((self.rows, self.cols), dtype=np.uint8)

    def xy_to_rc(self, x, y):
        r = int(math.floor((float(x) - self.origin_x) / self.res))
        c = int(math.floor((float(y) - self.origin_y) / self.res))
        return r, c

    def rc_to_xy(self, r, c):
        x = self.origin_x + (int(r) + 0.5) * self.res
        y = self.origin_y + (int(c) + 0.5) * self.res
        return x, y

    def in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def sample(self, x, y):
        r, c = self.xy_to_rc(x, y)
        if not self.in_bounds(r, c):
            return OCC_UNK
        return float(self.occ[r, c])

    def stamp_disk(self, x, y, radius, value, src):
        if x is None or y is None:
            return
        rad = max(self.res, float(radius))
        r0, c0 = self.xy_to_rc(x - rad, y - rad)
        r1, c1 = self.xy_to_rc(x + rad, y + rad)
        r0 = max(0, r0)
        c0 = max(0, c0)
        r1 = min(self.rows - 1, r1)
        c1 = min(self.cols - 1, c1)
        r2 = rad * rad
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                px, py = self.rc_to_xy(r, c)
                if (px - x) * (px - x) + (py - y) * (py - y) <= r2:
                    if value >= float(self.occ[r, c]):
                        self.occ[r, c] = value
                        self.src[r, c] = src

    def stamp_cell(self, r, c, value, src):
        if not self.in_bounds(r, c):
            return
        if value >= float(self.occ[r, c]):
            self.occ[r, c] = value
            self.src[r, c] = src
        elif value < OCC_UNK and self.src[r, c] == SRC_UNK:
            self.occ[r, c] = value
            self.src[r, c] = src

    def dilate_occupied(self, k):
        k = int(k)
        if k <= 0:
            return
        mask = self.occ >= OCC_THR
        ys, xs = np.nonzero(mask)
        extra = np.zeros_like(mask)
        for r, c in zip(ys, xs):
            extra[max(0, r - k):min(self.rows, r + k + 1),
                  max(0, c - k):min(self.cols, c + k + 1)] = True
        new = extra & ~mask
        self.occ[new] = np.maximum(self.occ[new], np.float32(OCC_BLOCK))
        self.src[new] = np.maximum(self.src[new], SRC_RUNWAY)

    def occupied_mask(self):
        return self.occ >= OCC_THR

    def clearance_field(self):
        """Metres to nearest occupied cell. Tiny grid: brute-force is honest and cheap."""
        mask = self.occupied_mask()
        occ_idx = np.argwhere(mask)
        dist = np.full((self.rows, self.cols), 8.0, dtype=np.float32)
        if occ_idx.size == 0:
            return dist
        rr = np.arange(self.rows)[:, None]
        cc = np.arange(self.cols)[None, :]
        for r, c in occ_idx:
            d = np.hypot((rr - r) * self.res, (cc - c) * self.res)
            dist = np.minimum(dist, d.astype(np.float32))
        dist[mask] = 0.0
        return dist

    def sample_clearance(self, x, y, field=None):
        field = self.clearance_field() if field is None else field
        r, c = self.xy_to_rc(x, y)
        if not self.in_bounds(r, c):
            return 0.15
        return float(field[r, c])

    def center_clearance_m(self, field=None):
        """Forward range (m) to the first occupied cell in |y|<0.28 m.

        Open corridor returns a large number so cruise is full-speed; DWA
        still scores side clearance on each rollout. Mixing the distance
        transform into this number made a chair 0.7 m left look like a wall.
        """
        x_hit = None
        for r in range(self.rows):
            for c in range(self.cols):
                x, y = self.rc_to_xy(r, c)
                if abs(y) > 0.28 or x < 0.20 or x > 1.8:
                    continue
                if self.occ[r, c] >= OCC_THR:
                    x_hit = x if x_hit is None else min(x_hit, x)
        if x_hit is not None:
            return max(0.0, float(x_hit))
        return 8.0

    def open_bearing_y(self):
        """Y of the y-slice with the farthest free range — gap to commit toward."""
        best_y, best_x = 0.0, -1.0
        for c in range(self.cols):
            x_end = 0.0
            for r in range(self.rows):
                x, y = self.rc_to_xy(r, c)
                if x < 0.15:
                    continue
                if self.occ[r, c] >= OCC_THR:
                    break
                x_end = x
            if x_end > best_x:
                best_x = x_end
                best_y = self.rc_to_xy(0, c)[1]
        return best_y, best_x

    def column_costs(self, n=9):
        """Legacy 9-bin costs (0 open .. 2 blocked) for the existing panel field."""
        costs = []
        y0, y1 = self.origin_y, self.origin_y + self.cols * self.res
        for j in range(n):
            ya = y0 + (y1 - y0) * j / n
            yb = y0 + (y1 - y0) * (j + 1) / n
            acc, cnt = 0.0, 0
            for r in range(self.rows):
                x, _ = self.rc_to_xy(r, 0)
                if x < 0.2 or x > 2.2:
                    continue
                for c in range(self.cols):
                    _, y = self.rc_to_xy(r, c)
                    if ya <= y < yb:
                        acc += float(self.occ[r, c])
                        cnt += 1
            avg = acc / max(1, cnt)
            costs.append(round(min(2.0, max(0.0, (avg - OCC_FREE) * 2.2)), 3))
        return costs

    def quantize(self):
        """0 unknown, 1 free, 2 runway-block, 3 yolo-block. Flat row-major."""
        q = np.zeros((self.rows, self.cols), dtype=np.uint8)
        q[self.occ < 0.35] = 1
        q[(self.occ >= OCC_THR) & (self.src == SRC_RUNWAY)] = 2
        q[(self.occ >= OCC_THR) & (self.src == SRC_YOLO)] = 3
        q[(self.occ >= OCC_THR) & (self.src == SRC_UNK)] = 2
        return q.flatten().tolist()

    def meta(self):
        return {
            'cols': self.cols, 'rows': self.rows, 'res': self.res,
            'origin_x': self.origin_x, 'origin_y': self.origin_y,
        }


def _grid_from_cfg(cfg):
    cols = cfg_i(cfg, 'grid_cols', 24)
    rows = cfg_i(cfg, 'grid_rows', 40)
    res = cfg_f(cfg, 'grid_res_m', 0.12)
    behind = cfg_f(cfg, 'grid_behind_m', 0.48)
    return Grid(cols=cols, rows=rows, res=res, origin_x=-behind)


def _stamp_runway(grid, col_open, cfg):
    """9-bin floor prior: free out to estimated range; blocked band if runway dies."""
    if not col_open:
        return
    n = len(col_open)
    block_thr = cfg_f(cfg, 'runway_block', 0.18)
    y0 = grid.origin_y
    yspan = grid.cols * grid.res
    for j, op in enumerate(col_open):
        op = max(0.0, min(1.0, float(op)))
        ya = y0 + yspan * j / n
        yb = y0 + yspan * (j + 1) / n
        free_to = 0.40 + op * 4.2
        for r in range(grid.rows):
            for c in range(grid.cols):
                x, y = grid.rc_to_xy(r, c)
                if y < ya or y >= yb or x < 0.05:
                    continue
                if x <= free_to:
                    if grid.src[r, c] == SRC_UNK:
                        grid.occ[r, c] = OCC_FREE
                        grid.src[r, c] = SRC_FREE
        if op < block_thr:
            # no floor texture ahead in this wedge → treat as a close wall
            x_hit = 0.45 + op * 1.2
            for r in range(grid.rows):
                for c in range(grid.cols):
                    x, y = grid.rc_to_xy(r, c)
                    if y < ya or y >= yb:
                        continue
                    if x_hit <= x <= x_hit + 0.9:
                        grid.stamp_cell(r, c, OCC_BLOCK, SRC_RUNWAY)


def _stamp_floor_mask(grid, floor_mask, cfg, frame_w, frame_h):
    if floor_mask is None:
        return
    m = np.asarray(floor_mask)
    if m.ndim != 2 or m.size == 0:
        return
    mh, mw = m.shape
    ys, xs = np.nonzero(m > 0)
    if ys.size == 0:
        return
    # subsample — 80x60 is already small; cap work
    step = max(1, ys.size // 900)
    u = (xs[::step] + 0.5) * (frame_w / mw)
    v = (ys[::step] + 0.5) * (frame_h / mh)
    gx, gy = pixel_to_ground_arr(u, v, cfg, frame_w, frame_h)
    for x, y in zip(gx, gy):
        if not np.isfinite(x):
            continue
        r, c = grid.xy_to_rc(x, y)
        if grid.in_bounds(r, c) and grid.src[r, c] == SRC_UNK:
            grid.occ[r, c] = OCC_FREE
            grid.src[r, c] = SRC_FREE


def _track_radius(cls, cfg):
    r = CLASS_R.get(cls, 0.26)
    if cls == 'person':
        r *= cfg_f(cfg, 'auto_person_margin', 2.0)
    return r + cfg_f(cfg, 'occ_inflate_m', 0.18) * 0.35


def project_track(tr, cfg, frame_w, frame_h):
    """Foot of the box onto the ground plane. Sets tr.gx, tr.gy."""
    u = 0.5 * (tr.x1 + tr.x2)
    v = tr.y2
    pt = pixel_to_ground(u, v, cfg, frame_w, frame_h)
    if pt is None:
        # box in the sky / above horizon: ignore for occupancy
        tr.gx, tr.gy = None, None
        return None
    x, y = pt
    if tr.gx is not None:
        tr.gvx = 0.6 * tr.gvx + 0.4 * (x - tr.gx)
        tr.gvy = 0.6 * tr.gvy + 0.4 * (y - tr.gy)
    tr.gx, tr.gy = x, y
    return pt


def _stamp_tracks(grid, tracks, cfg, frame_w, frame_h):
    for tr in tracks:
        pt = project_track(tr, cfg, frame_w, frame_h)
        if pt is None:
            continue
        x, y = pt
        rad = _track_radius(tr.cls, cfg)
        grid.stamp_disk(x, y, rad, OCC_BLOCK, SRC_YOLO)
        # also stamp bottom-left / bottom-right so wide boxes aren't a dot
        p_l = pixel_to_ground(tr.x1, tr.y2, cfg, frame_w, frame_h)
        p_r = pixel_to_ground(tr.x2, tr.y2, cfg, frame_w, frame_h)
        if p_l is not None:
            grid.stamp_disk(p_l[0], p_l[1], rad * 0.7, OCC_BLOCK, SRC_YOLO)
        if p_r is not None:
            grid.stamp_disk(p_r[0], p_r[1], rad * 0.7, OCC_BLOCK, SRC_YOLO)


def build_local(cfg, frame_w, frame_h, col_open, tracks, floor_mask=None):
    """Return a body-frame Grid for this camera frame. No world fusion."""
    grid = _grid_from_cfg(cfg)
    _stamp_runway(grid, col_open, cfg)
    _stamp_floor_mask(grid, floor_mask, cfg, frame_w, frame_h)
    _stamp_tracks(grid, tracks or [], cfg, frame_w, frame_h)
    k = int(round(cfg_f(cfg, 'occ_inflate_m', 0.18) / max(1e-6, grid.res)))
    grid.dilate_occupied(max(1, k))
    return grid
