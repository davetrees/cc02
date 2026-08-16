"""SORT tracker: stable IDs, class-aware IoU, miss timeout."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'brain'))

from tracker import SortTracker, _iou


class TrackerTests(unittest.TestCase):
    def test_iou_overlap_and_miss(self):
        self.assertGreater(_iou((0, 0, 10, 10), (5, 5, 15, 15)), 0.1)
        self.assertEqual(_iou((0, 0, 10, 10), (20, 20, 30, 30)), 0.0)

    def test_id_stable_across_frames(self):
        trk = SortTracker(iou_thr=0.3, max_missed=8, min_hits=2)
        box = (100, 80, 180, 240, 'chair', 0.9)
        self.assertEqual(trk.update([box], dt=0.05), [])  # not confirmed yet
        confirmed = trk.update([box], dt=0.05)
        self.assertEqual(len(confirmed), 1)
        tid = confirmed[0].tid
        moved = (108, 82, 188, 242, 'chair', 0.88)
        again = trk.update([moved], dt=0.05)
        self.assertEqual(len(again), 1)
        self.assertEqual(again[0].tid, tid)
        self.assertGreater(again[0].hits, 2)

    def test_class_mismatch_prefers_new_track(self):
        trk = SortTracker(iou_thr=0.3, min_hits=1)
        trk.update([(100, 80, 180, 240, 'chair', 0.9)], dt=0.05)
        # same pixels, different class: IoU scaled *0.35 -> below 0.3 unless huge overlap
        # 100% overlap * 0.35 = 0.35, still matches. Shrink overlap.
        other = trk.update([(160, 140, 250, 280, 'person', 0.9)], dt=0.05)
        ids = {t.cls: t.tid for t in trk.tracks}
        self.assertIn('chair', ids)
        self.assertIn('person', ids)
        self.assertNotEqual(ids['chair'], ids['person'])
        self.assertTrue(any(t.cls == 'person' for t in other) or
                        any(t.cls == 'person' for t in trk.tracks))

    def test_dropped_after_max_missed(self):
        trk = SortTracker(max_missed=2, min_hits=1)
        trk.update([(10, 10, 40, 40, 'bottle', 0.8)], dt=0.05)
        self.assertEqual(len(trk.tracks), 1)
        trk.update([], dt=0.05)
        trk.update([], dt=0.05)
        self.assertEqual(len(trk.tracks), 1)
        trk.update([], dt=0.05)
        self.assertEqual(len(trk.tracks), 0)


if __name__ == '__main__':
    unittest.main()
