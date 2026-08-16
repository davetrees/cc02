"""DWA planner: cruise open floor, stop at a wall, turn around a side obstacle."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'brain'))

import occupancy as occmod
import planner


CFG = {
    'auto_cruise_us': 120.0,
    'auto_min_move_us': 110.0,
    'auto_hyst': 0.15,
    'auto_clear_open_m': 1.35,
    'auto_clear_stop_m': 0.42,
    'speed_mps_per_us': 0.0035,
    'wheelbase_m': 0.242,
    'steer_max_deg': 28.0,
    'auto_steer_us': 320.0,
    'dwa_horizon_s': 1.6,
    'dwa_dt': 0.10,
    'dwa_n_v': 6,
    'dwa_n_delta': 9,
    'dwa_w_progress': 1.35,
    'dwa_w_speed': 1.10,
    'dwa_w_clear': 1.80,
    'dwa_w_curve': 0.55,
    'dwa_w_smooth': 0.70,
    'dwa_w_gap': 0.45,
    'dwa_w_commit': 0.22,
}


def _free_grid():
    g = occmod.Grid()
    g.occ[:, :] = occmod.OCC_FREE
    g.src[:, :] = occmod.SRC_FREE
    return g


class PlannerTests(unittest.TestCase):
    def test_none_grid_is_stop(self):
        p = planner.plan(None, CFG, cap_us=150)
        self.assertEqual(p['throttle_us'], 1500)
        self.assertEqual(p['v'], 0.0)
        self.assertTrue(p['hit'])

    def test_open_floor_cruises_straight(self):
        p = planner.plan(_free_grid(), CFG, last_v=0.0, last_delta=0.0, cap_us=150)
        self.assertGreater(p['v'], 0.05)
        self.assertGreater(p['throttle_us'], 1500)
        self.assertFalse(p['hit'])
        self.assertLess(abs(p['delta']), 0.25)
        self.assertGreater(p['corridor'], 1.0)

    def test_wall_ahead_stops(self):
        g = _free_grid()
        for r in range(g.rows):
            for c in range(g.cols):
                x, _y = g.rc_to_xy(r, c)
                if 0.32 <= x <= 0.70:
                    g.occ[r, c] = occmod.OCC_BLOCK
                    g.src[r, c] = occmod.SRC_RUNWAY
        p = planner.plan(g, CFG, last_v=0.3, last_delta=0.0, cap_us=150)
        self.assertLessEqual(p['v'], 0.02)
        self.assertEqual(p['throttle_us'], 1500)

    def test_obstacle_on_left_steers_right(self):
        g = _free_grid()
        g.stamp_disk(0.80, 0.55, 0.24, occmod.OCC_BLOCK, occmod.SRC_YOLO)
        g.dilate_occupied(1)
        p = planner.plan(g, CFG, last_v=0.0, last_delta=0.0, cap_us=150)
        self.assertGreater(p['v'], 0.02)
        self.assertLess(p['delta'], 0.0)  # y is left; right turn is negative yaw
        self.assertLess(p['goal_y'], 0.0)

    def test_hysteresis_holds_committed_in_open_floor(self):
        g = _free_grid()
        first = planner.plan(g, CFG, last_v=0.0, last_delta=0.0, cap_us=150)
        second = planner.plan(
            g, CFG, last_v=first['v'], last_delta=first['delta'], cap_us=150)
        self.assertGreater(second['v'], 0.05)
        self.assertAlmostEqual(second['v'], first['v'], delta=0.08)
        self.assertAlmostEqual(second['delta'], first['delta'], delta=0.08)

    def test_bicycle_rollout_turns_left(self):
        straight = planner.bicycle_rollout(0.4, 0.0, 0.242, 1.0, 0.1)
        left = planner.bicycle_rollout(0.4, 0.4, 0.242, 1.0, 0.1)
        self.assertGreater(straight[-1][0], 0.3)
        self.assertLess(abs(straight[-1][1]), 0.05)
        self.assertGreater(left[-1][1], 0.05)


if __name__ == '__main__':
    unittest.main()
