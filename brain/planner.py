"""Short-horizon DWA / bicycle lattice in the body-frame occupancy grid.

Runs inside the 20 Hz AUTO tick. Samples (v, steer) pairs, rolls out ~1.6 s,
rejects collisions, scores progress / clearance / smoothness, and commits to
a path until a challenger beats it by hysteresis — so open floor is a cruise,
not a twitch.
"""
from __future__ import annotations

import math

import occupancy as occmod


def cfg_f(cfg, key, default):
    try:
        return float(cfg.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def bicycle_rollout(v, delta, wheelbase, horizon_s, dt):
    """Body-frame poses (x, y, yaw) starting at origin, yaw=0 = straight ahead."""
    n = max(1, int(round(float(horizon_s) / max(1e-3, dt))))
    L = max(0.12, float(wheelbase))
    x = y = th = 0.0
    pts = []
    td = math.tan(max(-1.2, min(1.2, float(delta))))
    for _ in range(n):
        th += (v * td / L) * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        pts.append((x, y, th))
    return pts


def traj_hit_and_clearance(grid, pts, field):
    """Return (hit, min_clearance_m, x_end). Out-of-grid ahead counts as unknown, not hit."""
    dmin = 8.0
    hit = False
    x_end = 0.0
    for x, y, _th in pts:
        x_end = x
        r, c = grid.xy_to_rc(x, y)
        if not grid.in_bounds(r, c):
            if x > 0.2 and abs(y) < 0.35:
                dmin = min(dmin, 0.25)
            continue
        if grid.occ[r, c] >= occmod.OCC_THR:
            hit = True
            dmin = 0.0
            break
        dmin = min(dmin, float(field[r, c]))
    return hit, dmin, x_end


def us_to_v(us_dev, cfg):
    return cfg_f(cfg, 'speed_mps_per_us', 0.0035) * float(us_dev)


def v_to_us(v, cfg):
    k = max(1e-6, cfg_f(cfg, 'speed_mps_per_us', 0.0035))
    return v / k


def steer_to_delta(steer_us, cfg):
    span = max(40.0, cfg_f(cfg, 'auto_steer_us', 320.0))
    mx = math.radians(cfg_f(cfg, 'steer_max_deg', 28.0))
    return max(-mx, min(mx, (float(steer_us) - 1500.0) / span * mx))


def delta_to_steer(delta, cfg):
    span = max(40.0, cfg_f(cfg, 'auto_steer_us', 320.0))
    mx = math.radians(cfg_f(cfg, 'steer_max_deg', 28.0))
    if mx < 1e-3:
        return 1500
    return int(round(1500.0 + (delta / mx) * span))


def score_trajectory(v, delta, pts, hit, clearance, x_end, goal_y, last_v, last_delta,
                     committed, cfg):
    """Higher is better. Hard-reject collisions with a huge negative."""
    if hit:
        return -1.0e6
    w_prog = cfg_f(cfg, 'dwa_w_progress', 1.35)
    w_spd = cfg_f(cfg, 'dwa_w_speed', 1.10)
    w_clr = cfg_f(cfg, 'dwa_w_clear', 1.80)
    w_curv = cfg_f(cfg, 'dwa_w_curve', 0.55)
    w_jerk = cfg_f(cfg, 'dwa_w_smooth', 0.70)
    w_gap = cfg_f(cfg, 'dwa_w_gap', 0.45)
    w_commit = cfg_f(cfg, 'dwa_w_commit', 0.22)
    clr_n = max(0.0, min(1.0, clearance / 1.20))
    score = 0.0
    score += w_prog * x_end
    score += w_spd * v * (0.25 + 0.75 * clr_n)
    score += w_clr * min(clearance, 1.6)
    # in open floor, turning is almost free-to-penalize so we stay committed straight;
    # in clutter, curvature is allowed if it buys clearance
    curve_w = w_curv * (0.25 + 0.75 * (1.0 - clr_n))
    score -= curve_w * abs(delta)
    score -= w_jerk * (abs(delta - last_delta) + 0.45 * abs(v - last_v))
    y_end = pts[-1][1] if pts else 0.0
    score += w_gap * (1.0 - min(1.0, abs(y_end - goal_y) / 1.1))
    if committed:
        score += w_commit
    if v > 0.02 and clearance < cfg_f(cfg, 'auto_clear_stop_m', 0.42):
        score -= 2.5
    return score


def adaptive_vmax(clearance, v_cruise, cfg):
    open_m = cfg_f(cfg, 'auto_clear_open_m', 1.35)
    stop_m = cfg_f(cfg, 'auto_clear_stop_m', 0.42)
    if clearance >= open_m:
        scale = 1.0
    elif clearance <= stop_m:
        scale = 0.0
    else:
        scale = (clearance - stop_m) / max(1e-6, open_m - stop_m)
    return v_cruise * scale, scale


def plan(grid, cfg, last_v=0.0, last_delta=0.0, cap_us=150.0):
    """Sample DWA. Returns dict with v, delta, steer_us, throttle_us, traj, score, clearance, scale.

    last_v / last_delta are the committed command (body-frame), used for smoothness
    and hysteresis so we do not flip-flop between equal gaps.
    """
    if grid is None:
        return {
            'v': 0.0, 'delta': 0.0, 'steer_us': 1500, 'throttle_us': 1500,
            'traj': [], 'score': -1.0e6, 'clearance': 0.0, 'scale': 0.0, 'hit': True,
        }

    field = grid.clearance_field()
    corridor = grid.center_clearance_m(field)
    goal_y, _gap = grid.open_bearing_y()

    cruise_us = min(float(cap_us), cfg_f(cfg, 'auto_cruise_us', 120.0))
    v_cruise = max(0.05, us_to_v(cruise_us, cfg))
    v_cap, scale = adaptive_vmax(corridor, v_cruise, cfg)
    min_move_us = cfg_f(cfg, 'auto_min_move_us', 110.0)
    v_min_move = us_to_v(min_move_us, cfg)

    horizon = cfg_f(cfg, 'dwa_horizon_s', 1.6)
    dt = cfg_f(cfg, 'dwa_dt', 0.10)
    L = cfg_f(cfg, 'wheelbase_m', 0.242)
    dmax = math.radians(cfg_f(cfg, 'steer_max_deg', 28.0))
    last_v = max(0.0, min(float(last_v), max(v_cap, 0.0)))
    last_delta = max(-dmax, min(dmax, float(last_delta)))
    n_v = max(3, int(cfg_f(cfg, 'dwa_n_v', 6)))
    n_d = max(5, int(cfg_f(cfg, 'dwa_n_delta', 9)))

    # speeds: always include 0; if we can move, include min-move and up to v_cap
    speeds = [0.0]
    if v_cap > 0.02:
        lo = min(v_min_move, v_cap)
        hi = max(lo, v_cap)
        for i in range(n_v):
            if n_v == 1:
                speeds.append(hi)
            else:
                speeds.append(lo + (hi - lo) * i / (n_v - 1))
    speeds = sorted(set(round(s, 4) for s in speeds))

    deltas = [(-dmax + 2.0 * dmax * i / (n_d - 1)) for i in range(n_d)]
    # always re-evaluate the committed pair so hysteresis has a real number
    candidates = [(sv, sd) for sv in speeds for sd in deltas]
    candidates.append((last_v, last_delta))

    best = None
    committed_score = None
    for v, delta in candidates:
        delta = max(-dmax, min(dmax, delta))
        pts = bicycle_rollout(v, delta, L, horizon, dt)
        hit, clr, x_end = traj_hit_and_clearance(grid, pts, field)
        is_commit = (abs(v - last_v) < 0.03 and abs(delta - last_delta) < 0.04)
        sc = score_trajectory(v, delta, pts, hit, clr, x_end, goal_y,
                              last_v, last_delta, is_commit, cfg)
        rec = {
            'v': v, 'delta': delta, 'pts': pts, 'score': sc,
            'clearance': clr, 'hit': hit, 'x_end': x_end,
        }
        if is_commit:
            committed_score = sc
        if best is None or sc > best['score']:
            best = rec

    hyst = cfg_f(cfg, 'auto_hyst', 0.15)
    # hysteresis is for cruise, not for staying parked: a standing start must
    # be allowed to pick a moving path as soon as one is clearly better.
    if (committed_score is not None and best is not None and last_v > 0.05):
        margin = hyst * (2.0 + abs(committed_score) * 0.15)
        if best['score'] < committed_score + margin:
            # re-pick committed if it is still feasible
            pts = bicycle_rollout(last_v, last_delta, L, horizon, dt)
            hit, clr, x_end = traj_hit_and_clearance(grid, pts, field)
            if not hit and last_v >= -0.01:
                best = {
                    'v': last_v, 'delta': last_delta, 'pts': pts,
                    'score': committed_score, 'clearance': clr, 'hit': hit,
                    'x_end': x_end,
                }

    if best is None or best['hit'] or best['v'] <= 0.02:
        steer_us = 1500
        throttle_us = 1500
        v_out, d_out = 0.0, 0.0
        traj = [(p[0], p[1]) for p in (best['pts'] if best else [])]
        sc = best['score'] if best else -1.0e6
        clr = corridor
        scale_out = 0.0
    else:
        v_out, d_out = best['v'], best['delta']
        us = v_to_us(v_out, cfg)
        if 0 < us < min_move_us:
            us = min_move_us if v_cap >= v_min_move * 0.5 else 0.0
        us = min(float(cap_us), us)
        throttle_us = 1500 + int(round(us)) if us > 0 else 1500
        steer_us = max(1000, min(2000, delta_to_steer(d_out, cfg)))
        traj = [(round(p[0], 3), round(p[1], 3)) for p in best['pts']]
        sc = best['score']
        clr = best['clearance']
        scale_out = scale

    return {
        'v': v_out,
        'delta': d_out,
        'steer_us': int(steer_us),
        'throttle_us': int(throttle_us),
        'traj': traj,
        'score': sc,
        'clearance': clr,
        'scale': scale_out,
        'hit': bool(best['hit']) if best else True,
        'goal_y': goal_y,
        'corridor': corridor,
    }
