"""20Hz decision loop -> (steer_us, throttle_us). Also dead-reckoned pose + breadcrumbs.

MANUAL: user passthrough. ASSIST: passthrough but forward throttle cut to 1500 on
collision (reverse allowed). AUTO: 9-column cost-map cruiser (below). RTH:
breadcrumb replay in reverse (BEST-EFFORT dead reckoning). ESTOP: 1500/1500.

AUTO cruiser (rewritten 2026-08-16):
- Cost map: 9 columns fusing (a) the Canny floor-edge openness from vision
  (st.col_open, 0..1 per column, 1 = open) and (b) ALL YOLO detections
  (st.boxes_all - the yolo_classes collision filter is deliberately ignored
  here) projected onto the columns their x-span overlaps, weighted by box
  area fraction; person boxes get auto_person_margin (2x) width safety margin.
- Steering: P toward the lowest-cost column, with hysteresis - the target
  column only changes when the challenger beats the current target's cost by
  auto_hyst, so the car does not flip-flop between near-equal gaps.
- Speed: scales with center clearance (1 - worst cost of the middle 3
  columns): >= auto_open_thr -> full cruise (auto_cruise_us, capped by
  max_speed); linear taper in between; <= auto_block_thr -> stop.
- Escape: blocked (or st.collision) continuously > auto_block_time_s -> stop
  auto_stop_time_s, reverse auto_reverse_time_s steering away from the
  costlier side, resume cruise. If a person box is the obstruction: HOLD at
  neutral until the person clears - NEVER reverse-escape at a person.
- Stuck: forward commanded > stuck_time_s while IMU accel variance over a 2s
  window is ~0 (chassis not vibrating = not actually moving) -> same escape.
- Safety: if vision data is older than 0.5s AUTO holds neutral (no blind
  driving, no blind escapes). Reverse deviation is also capped by max_speed.

The cost map + would-be target/steer are computed every tick in every mode and
published (st.auto_costs / auto_target / auto_steer_us / auto_state /
auto_accel_var) so the bench can watch the cruiser's choices without AUTO
engaged.
"""
import math
import threading
import time
from collections import deque

import protocol as P

STEER_P_US = 300.0      # AUTO: full-scale steering deviation
RTH_KP_US = 350.0       # rad -> us
RTH_THROTTLE = 1550
N_COLS = 9
MID = N_COLS // 2
WP_REACHED_M = 0.3
SPEED_K = 0.004         # m/s per us of throttle deviation (placeholder model)
INPUT_TIMEOUT_S = 0.5
VISION_STALE_S = 0.5


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Autopilot(threading.Thread):
    def __init__(self, state, mapping):
        super().__init__(daemon=True, name='autopilot')
        self.state = state
        self.mapping = mapping
        self._last_crumb = 0.0
        self._rth_wps = []
        self._prev_mode = P.MODE_MANUAL
        self._gz_bias = 0.0
        self._bias_logged = False
        # AUTO cruiser state
        self._target = MID
        self._auto_state = 'CRUISE'
        self._state_t = 0.0             # entry time of ESCAPE_* states
        self._blocked_since = None
        self._fwd_since = None
        self._person_clear_since = None
        self._escape_dir = 1            # +1 steer right, -1 steer left
        self._accel_buf = deque(maxlen=40)  # 2s @ 20Hz for stuck detection
        self._last_stale_log = 0.0

    def run(self):
        st = self.state
        last = time.monotonic()
        next_t = last
        while st.running:
            next_t += 0.05  # 20 Hz
            now = time.monotonic()
            dt = now - last
            last = now
            try:
                self._step(dt, now)
            except Exception as e:
                st.log(f'autopilot: error {e}')
                st.out_steer_us, st.out_throttle_us = 1500, 1500
            sl = next_t - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            else:
                next_t = time.monotonic()

    def _step(self, dt, now):
        st = self.state
        # input watchdog: no client input for 500ms -> neutral
        if now - st.last_input_time > INPUT_TIMEOUT_S:
            st.user_steer = 0.0
            st.user_throttle = 0.0

        cap_us = 500.0 * float(st.config.get('max_speed', 0.30))
        user_steer_us = 1500 + int(max(-500, min(500, st.user_steer * 500)))
        thr_dev = max(-cap_us, min(cap_us, st.user_throttle * 500.0))
        user_throttle_us = 1500 + int(thr_dev)

        mode = st.mode

        # instant override: meaningful user input during AUTO/RTH -> MANUAL
        if mode in (P.MODE_AUTO, P.MODE_RTH) and not st.estop:
            othr = float(st.config.get('override_thr', 0.3))
            if abs(st.user_steer) > othr or abs(st.user_throttle) > othr:
                st.log(f'override: user input steer={st.user_steer:+.2f} '
                       f'throttle={st.user_throttle:+.2f} -> MANUAL')
                st.set_mode(P.MODE_MANUAL)
                mode = st.mode

        # accel window for stuck detection (cleared if telemetry stale)
        t = st.telem
        if t and now - st.last_telem_time < 0.5:
            self._accel_buf.append((float(t.get('ax', 0.0)),
                                    float(t.get('ay', 0.0)),
                                    float(t.get('az', 0.0))))
        else:
            self._accel_buf.clear()
        st.auto_accel_var = self._accel_var()

        # cost map + would-be steering target: computed every tick in every
        # mode (bench observability); only AUTO acts on it
        self._update_cost_map(now)

        # FSM only lives while AUTO is engaged
        if mode != P.MODE_AUTO:
            self._auto_state = 'CRUISE'
            self._blocked_since = None
            self._fwd_since = None
            st.auto_state = self._auto_state

        collision_stop = bool(st.config.get('collision_stop', True))
        col = st.collision and collision_stop

        if st.estop:
            steer_us, throttle_us = 1500, 1500
        elif mode == P.MODE_MANUAL:
            steer_us, throttle_us = user_steer_us, user_throttle_us
        elif mode == P.MODE_ASSIST:
            steer_us, throttle_us = user_steer_us, user_throttle_us
            if col and throttle_us > 1500:  # forward only; reverse allowed
                throttle_us = 1500
        elif mode == P.MODE_AUTO:
            steer_us, throttle_us = self._auto(cap_us, now)
        elif mode == P.MODE_RTH:
            steer_us, throttle_us = self._rth(col)
        else:
            steer_us, throttle_us = 1500, 1500

        # Pi second-layer anti-tip (default OFF). Past counter_steer_deg of
        # roll, add a proportional steer offset. counter_steer_dir -1 matches
        # firmware ARS_DIR -1: bias is |lean| past the gate, then
        # steer += dir * sign(roll) * bias, which yaws AWAY from the falling
        # side (inertia out of tilt). +1 would fight the ESP.
        t2 = st.telem
        cs_on = bool(st.config.get("counter_steer_enable", False))
        if cs_on and t2 and not st.estop and throttle_us > 1520:
            roll_deg = math.degrees(float(t2.get("roll", 0.0)))
            cthr = float(st.config.get("counter_steer_deg", 30))
            # >70 deg = flopped, not saveable; also never act on an un-zeroed AH
            if cthr < abs(roll_deg) < 70:
                k = float(st.config.get("counter_steer_us_per_deg", 15))
                cdir = int(st.config.get("counter_steer_dir", -1))
                bias = min(400.0, k * (abs(roll_deg) - cthr))
                steer_us += cdir * (1 if roll_deg > 0 else -1) * bias

        tgt_t = max(1000, min(2000, int(throttle_us)))
        tgt_s = max(1000, min(2000, int(steer_us)))
        # smooth throttle ramp: ease toward target instead of on/off. Accel is
        # slew-limited; decel/brake is faster (safety); estop/neutral is instant.
        cur = getattr(st, "out_throttle_us", 1500)
        if st.estop or tgt_t == 1500:
            st.out_throttle_us = 1500
        elif getattr(st, "user_direct", False) and mode == P.MODE_MANUAL:
            st.out_throttle_us = tgt_t   # DIRECT: raw, no ramp (left stick)
        else:
            up = float(st.config.get("throttle_slew_up", 10))    # us/tick (~200us/s @20Hz)
            dn = float(st.config.get("throttle_slew_dn", 45))    # us/tick faster to back off
            toward = tgt_t - 1500
            curd = cur - 1500
            # moving away from neutral (accelerating) uses up-rate; toward uses dn-rate
            accel = abs(tgt_t - 1500) > abs(cur - 1500) and (toward * curd >= 0)
            step = up if accel else dn
            delta = tgt_t - cur
            st.out_throttle_us = int(cur + max(-step, min(step, delta)))
        # steer eases too, but quicker so it stays responsive
        curs = getattr(st, "out_steer_us", 1500)
        if getattr(st, "user_direct", False) and mode == P.MODE_MANUAL:
            st.out_steer_us = tgt_s      # DIRECT steer
        else:
            ss = float(st.config.get("steer_slew", 60))
            st.out_steer_us = int(curs + max(-ss, min(ss, tgt_s - curs)))

        # ---- dead reckoning (BEST-EFFORT) ----
        t = st.telem
        yaw_rate = float(t.get('gz', 0.0)) if t else 0.0
        if t and 'yaw_rate' in t:
            yaw_rate = float(t['yaw_rate'])
        # gyro auto-zero: learn gz bias whenever throttle is neutral and rotation
        # is near-still (re-zeros within ~2-3s every time the car stops)
        if st.out_throttle_us == 1500 and abs(yaw_rate - self._gz_bias) < 0.05:
            self._gz_bias += 0.02 * (yaw_rate - self._gz_bias)
            if not self._bias_logged and abs(self._gz_bias) > 1e-4:
                self._bias_logged = True
                st.log(f'autopilot: gz bias learned {self._gz_bias:+.4f} rad/s')
        yaw_rate -= self._gz_bias
        st.pose_h = wrap_pi(st.pose_h + yaw_rate * dt)
        v = SPEED_K * (st.out_throttle_us - 1500)
        st.pose_x += v * math.cos(st.pose_h) * dt
        st.pose_y += v * math.sin(st.pose_h) * dt

        # breadcrumbs every 0.5s during all driving
        if now - self._last_crumb >= 0.5:
            self._last_crumb = now
            self.mapping.add_crumb(st.pose_x, st.pose_y, st.pose_h)

        # RTH entry/exit bookkeeping
        if mode == P.MODE_RTH and self._prev_mode != P.MODE_RTH:
            self._rth_wps = list(self.mapping.crumbs)[::-1]  # newest first
            st.log(f'RTH: replaying {len(self._rth_wps)} breadcrumbs (BEST-EFFORT)')
        self._prev_mode = mode
        st.rth_remaining = len(self._rth_wps) if mode == P.MODE_RTH else 0

    # ---------------- AUTO cruiser ----------------
    def _update_cost_map(self, now):
        """9-column cost map (0=open .. 2=very blocked) + hysteresis target."""
        st = self.state
        cfg = st.config
        if now - st.vision_time > VISION_STALE_S:
            st.auto_costs = [1.0] * N_COLS  # vision stale: treat all blocked
        else:
            opens = list(st.col_open)
            if len(opens) != N_COLS:
                opens = [0.0] * N_COLS
            floor_w = float(cfg.get('auto_floor_weight', 1.0))
            gain = float(cfg.get('auto_yolo_gain', 3.0))
            margin = float(cfg.get('auto_person_margin', 2.0))
            w = float(max(1, st.frame_w))
            h = float(max(1, st.frame_h))
            area = w * h
            colw = w / N_COLS
            costs = [floor_w * (1.0 - o) for o in opens]
            for (x1, y1, x2, y2, cls, conf) in list(st.boxes_all):
                bw, bh = x2 - x1, y2 - y1
                if bw <= 0 or bh <= 0:
                    continue
                frac = (bw * bh) / area
                cx = (x1 + x2) / 2.0
                half = bw / 2.0 * (margin if cls == 'person' else 1.0)
                for j in range(N_COLS):
                    ov = min(cx + half, (j + 1) * colw) - max(cx - half, j * colw)
                    if ov > 0:
                        costs[j] += gain * frac * (ov / colw)
            st.auto_costs = [round(min(2.0, c), 3) for c in costs]

        costs = st.auto_costs
        best = min(range(N_COLS), key=costs.__getitem__)
        if costs[self._target] - costs[best] > float(cfg.get('auto_hyst', 0.15)):
            self._target = best
        st.auto_target = self._target
        st.auto_steer_us = 1500 + int(STEER_P_US * (self._target - MID) / MID)

    def _person_obstruction(self):
        """A person box (any size >= auto_person_area, with safety margin)
        overlapping the center third of the frame."""
        st = self.state
        w = float(max(1, st.frame_w))
        h = float(max(1, st.frame_h))
        area = w * h
        min_area = float(st.config.get('auto_person_area', 0.02))
        margin = float(st.config.get('auto_person_margin', 2.0))
        for (x1, y1, x2, y2, cls, conf) in list(st.boxes_all):
            if cls != 'person':
                continue
            if ((x2 - x1) * (y2 - y1)) / area < min_area:
                continue
            cx = (x1 + x2) / 2.0
            half = (x2 - x1) / 2.0 * margin
            if cx + half > w / 3.0 and cx - half < 2.0 * w / 3.0:
                return True
        return False

    def _accel_var(self):
        """Sum of per-axis accel variances over the 2s window; None if the
        window isn't full yet (telemetry stale/short)."""
        b = self._accel_buf
        if len(b) < b.maxlen:
            return None
        n = float(len(b))
        tot = 0.0
        for i in range(3):
            s = sum(v[i] for v in b)
            s2 = sum(v[i] * v[i] for v in b)
            tot += max(0.0, s2 / n - (s / n) ** 2)
        return tot

    def _begin_escape(self, now, person, why):
        st = self.state
        self._blocked_since = None
        self._fwd_since = None
        self._state_t = now
        if person:
            st.log(f'auto: {why}, obstruction is a PERSON -> HOLD until clear '
                   '(never reverse at a person)')
            self._auto_state = 'HOLD_PERSON'
            self._person_clear_since = None
            return
        costs = st.auto_costs
        left, right = sum(costs[:MID]), sum(costs[MID + 1:])
        self._escape_dir = 1 if left > right else -1  # steer away from costlier side
        st.log(f'auto: {why} -> escape (stop '
               f'{float(st.config.get("auto_stop_time_s", 0.3)):.1f}s, reverse '
               f'{float(st.config.get("auto_reverse_time_s", 0.8)):.1f}s steering '
               f'{"right" if self._escape_dir > 0 else "left"})')
        self._auto_state = 'ESCAPE_STOP'

    def _auto(self, cap_us, now):
        st = self.state
        cfg = st.config

        # vision stale -> hold neutral; never drive or escape blind
        if now - st.vision_time > VISION_STALE_S:
            if now - self._last_stale_log > 5.0:
                self._last_stale_log = now
                st.log('auto: vision stale, holding neutral')
            self._auto_state = 'CRUISE'
            self._blocked_since = None
            self._fwd_since = None
            st.auto_state = self._auto_state
            return 1500, 1500

        costs = st.auto_costs
        steer_us = st.auto_steer_us

        # speed from center clearance (worst of the middle 3 columns)
        clearance = max(0.0, 1.0 - max(costs[MID - 1:MID + 2]))
        open_thr = float(cfg.get('auto_open_thr', 0.65))
        block_thr = float(cfg.get('auto_block_thr', 0.25))
        if clearance >= open_thr:
            scale = 1.0
        elif clearance <= block_thr:
            scale = 0.0
        else:
            scale = (clearance - block_thr) / max(1e-6, open_thr - block_thr)
        cruise = float(cfg.get('auto_cruise_us', 120))
        # ESC DEADBAND FLOOR (learned from field driving 2026-08-16): a forward
        # command below ~110us produces NO motion, which then trips the
        # stuck-detector into an endless false escape loop. Any non-zero cruise
        # commands at least auto_min_move_us; below that scale, stop outright.
        min_move = float(cfg.get('auto_min_move_us', 110))
        if scale <= 0.0:
            throttle_us = 1500
        else:
            dev = min(cap_us, max(min_move, cruise * scale))
            throttle_us = 1500 + int(dev)

        blocked = scale == 0.0 or st.collision
        person = self._person_obstruction()

        if self._auto_state == 'CRUISE':
            if blocked:
                throttle_us = 1500
                if self._blocked_since is None:
                    self._blocked_since = now
                if now - self._blocked_since > float(cfg.get('auto_block_time_s', 0.7)):
                    self._begin_escape(
                        now, person,
                        f'blocked {now - self._blocked_since:.1f}s '
                        f'(clearance {clearance:.2f}, collision {st.collision})')
            else:
                self._blocked_since = None
                # stuck: forward commanded but chassis not vibrating
                if throttle_us > 1505:
                    if self._fwd_since is None:
                        self._fwd_since = now
                    var = st.auto_accel_var
                    if (now - self._fwd_since > float(cfg.get('stuck_time_s', 2.0))
                            and var is not None
                            and var < float(cfg.get('stuck_accel_var', 0.02))):
                        self._begin_escape(
                            now, person,
                            f'STUCK (fwd {now - self._fwd_since:.1f}s, '
                            f'accel var {var:.4f})')
                else:
                    self._fwd_since = None

        if self._auto_state == 'HOLD_PERSON':
            steer_us, throttle_us = 1500, 1500
            if person:
                self._person_clear_since = None
            else:
                if self._person_clear_since is None:
                    self._person_clear_since = now
                elif now - self._person_clear_since > 0.5:
                    st.log('auto: person clear -> resume cruise')
                    self._auto_state = 'CRUISE'
                    self._blocked_since = None
        elif self._auto_state == 'ESCAPE_STOP':
            steer_us, throttle_us = 1500, 1500
            if person:
                st.log('auto: person appeared during escape -> HOLD')
                self._auto_state = 'HOLD_PERSON'
                self._person_clear_since = None
            elif now - self._state_t > float(cfg.get('auto_stop_time_s', 0.3)):
                self._auto_state = 'ESCAPE_REVERSE'
                self._state_t = now
        elif self._auto_state == 'ESCAPE_REVERSE':
            steer_us = 1500 + self._escape_dir * int(cfg.get('auto_escape_steer_us', 300))
            rev = min(float(cfg.get('auto_reverse_us', 140)), cap_us)
            throttle_us = 1500 - int(rev)
            if now - self._state_t > float(cfg.get('auto_reverse_time_s', 0.8)):
                st.log('auto: escape done -> resume cruise')
                self._auto_state = 'CRUISE'
                self._blocked_since = None
                self._fwd_since = None
                self._accel_buf.clear()

        st.auto_state = self._auto_state
        return steer_us, throttle_us

    # ---------------- RTH ----------------
    def _rth(self, col):
        st = self.state
        if col:
            return 1500, 1500
        while self._rth_wps:
            wp = self._rth_wps[0]
            dx = wp['x'] - st.pose_x
            dy = wp['y'] - st.pose_y
            if math.hypot(dx, dy) < WP_REACHED_M:
                self._rth_wps.pop(0)
                continue
            desired = math.atan2(dy, dx)
            err = wrap_pi(desired - st.pose_h)
            steer_us = 1500 + int(max(-500, min(500, RTH_KP_US * err)))
            return steer_us, RTH_THROTTLE
        # done
        st.log('RTH: breadcrumb list exhausted, back to MANUAL')
        st.set_mode(P.MODE_MANUAL)
        return 1500, 1500
