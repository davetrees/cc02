"""Body-frame occupancy: homography + this-frame grid, no pose / no world map."""
import inspect
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'brain'))

import occupancy as occmod
from tracker import Track


CFG = {
    'cam_height_m': 0.16,
    'cam_tilt_deg': 22.0,
    'cam_fx': 520.0,
    'cam_fy': 520.0,
    'cam_x_m': 0.08,
    'cam_y_m': 0.0,
    'occ_inflate_m': 0.18,
    'grid_cols': 24,
    'grid_rows': 40,
    'grid_res_m': 0.12,
    'grid_behind_m': 0.48,
    'runway_block': 0.18,
    'auto_person_margin': 2.0,
}


class OccupancyTests(unittest.TestCase):
    def test_build_local_has_no_pose(self):
        names = inspect.signature(occmod.build_local).parameters
        self.assertNotIn('pose', names)
        self.assertNotIn('pose_x', names)
        self.assertNotIn('pose_y', names)
        self.assertNotIn('heading', names)

    def test_pixel_to_ground_body_frame(self):
        mid = occmod.pixel_to_ground(320, 240, CFG, 640, 480)
        self.assertIsNotNone(mid)
        self.assertGreater(mid[0], 0.25)
        self.assertLess(abs(mid[1]), 0.08)
        left = occmod.pixel_to_ground(180, 360, CFG, 640, 480)
        right = occmod.pixel_to_ground(460, 360, CFG, 640, 480)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertGreater(left[1], right[1])  # y is left-positive
        near = occmod.pixel_to_ground(320, 430, CFG, 640, 480)
        far = occmod.pixel_to_ground(320, 300, CFG, 640, 480)
        self.assertIsNotNone(near)
        self.assertIsNotNone(far)
        self.assertLess(near[0], far[0])

    def test_open_runway_is_mostly_free(self):
        grid = occmod.build_local(CFG, 640, 480, [1.0] * 9, [], None)
        free = (grid.occ < occmod.OCC_THR).mean()
        self.assertGreater(free, 0.7)
        self.assertGreater(grid.center_clearance_m(), 1.0)

    def test_blocked_runway_stamps_a_wall(self):
        grid = occmod.build_local(CFG, 640, 480, [0.0] * 9, [], None)
        self.assertTrue(grid.occupied_mask().any())
        self.assertLess(grid.center_clearance_m(), 1.0)

    def test_yolo_box_stamps_occupied_disk(self):
        # foot of a chair near image bottom-center -> ground in front
        tr = Track(1, (280, 300, 360, 450, 'chair', 0.92))
        grid = occmod.build_local(CFG, 640, 480, [1.0] * 9, [tr], None)
        self.assertIsNotNone(tr.gx)
        self.assertGreater(tr.gx, 0.1)
        self.assertTrue((grid.src == occmod.SRC_YOLO).any())
        self.assertGreaterEqual(grid.sample(tr.gx, tr.gy), occmod.OCC_THR)

    def test_rebuild_does_not_remember_old_obstacles(self):
        tr = Track(1, (280, 300, 360, 450, 'chair', 0.92))
        g1 = occmod.build_local(CFG, 640, 480, [1.0] * 9, [tr], None)
        g2 = occmod.build_local(CFG, 640, 480, [1.0] * 9, [], None)
        self.assertTrue((g1.src == occmod.SRC_YOLO).any())
        self.assertFalse((g2.src == occmod.SRC_YOLO).any())


if __name__ == '__main__':
    unittest.main()
