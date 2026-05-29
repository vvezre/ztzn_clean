import unittest

import numpy as np

from vision_line_detection import (
    GuidanceBandTracker,
    find_vertical_bright_band,
    resolve_guidance_command,
)


class VisionLineDetectionTest(unittest.TestCase):
    def test_detects_center_of_wide_bright_band(self):
        gray = np.zeros((240, 320), dtype=np.uint8)
        gray[20:230, 145:177] = 255

        band = find_vertical_bright_band(gray, center_x=160)

        self.assertIsNotNone(band)
        self.assertAlmostEqual(band["center_x"], 160.5, delta=2.0)
        self.assertGreaterEqual(band["width"], 30)

    def test_ignores_thin_bright_line(self):
        gray = np.zeros((240, 320), dtype=np.uint8)
        gray[20:230, 158:163] = 255

        band = find_vertical_bright_band(gray, center_x=160)

        self.assertIsNone(band)

    def test_prefers_band_closest_to_center(self):
        gray = np.zeros((240, 320), dtype=np.uint8)
        gray[10:230, 40:70] = 255
        gray[10:230, 170:200] = 255

        band = find_vertical_bright_band(gray, center_x=190)

        self.assertIsNotNone(band)
        self.assertAlmostEqual(band["center_x"], 184.5, delta=2.0)

    def test_no_detection_sends_nothing_before_any_tracking(self):
        result = resolve_guidance_command(None, 0, was_tracking=False)

        self.assertEqual(result["mode"], "no_detection")
        self.assertFalse(result["send"])
        self.assertFalse(result["tracking"])
        self.assertIsNone(result["z_speed"])

    def test_no_detection_sends_single_zero_when_tracking_is_lost(self):
        result = resolve_guidance_command(None, 0, was_tracking=True)

        self.assertEqual(result["mode"], "tracking_lost")
        self.assertTrue(result["send"])
        self.assertFalse(result["tracking"])
        self.assertEqual(result["z_speed"], 0)

    def test_large_angle_uses_angle_based_correction(self):
        result = resolve_guidance_command(6, 14, was_tracking=False)

        self.assertEqual(result["mode"], "detected")
        self.assertTrue(result["send"])
        self.assertTrue(result["tracking"])
        self.assertEqual(result["z_speed"], -28)

    def test_visual_offset_correction_uses_offset_sign(self):
        result = resolve_guidance_command(12, 0, was_tracking=False)

        self.assertEqual(result["mode"], "detected")
        self.assertEqual(result["z_speed"], 12)

    def test_band_tracker_requires_consecutive_stable_centered_bands(self):
        tracker = GuidanceBandTracker(
            max_abs_offset=60,
            max_offset_jump=20,
            max_width_change=8,
            min_stable_frames=3,
        )
        line = {"mode": "bright_band", "offset": 12, "angle": 0, "width": 30}

        self.assertIsNone(tracker.update(line))
        self.assertIsNone(tracker.update(dict(line, offset=14, width=31)))
        accepted = tracker.update(dict(line, offset=15, width=30))

        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["offset"], 15)

    def test_band_tracker_rejects_offset_too_far_from_center(self):
        tracker = GuidanceBandTracker(max_abs_offset=60, min_stable_frames=1)

        accepted = tracker.update({"mode": "bright_band", "offset": 95, "angle": 0, "width": 30})

        self.assertIsNone(accepted)

    def test_band_tracker_accepts_far_offset_when_center_limit_disabled(self):
        tracker = GuidanceBandTracker(
            max_abs_offset=None,
            max_offset_jump=20,
            max_width_change=8,
            min_stable_frames=3,
        )
        line = {"mode": "bright_band", "offset": 120, "angle": 0, "width": 30}

        self.assertIsNone(tracker.update(line))
        self.assertIsNone(tracker.update(dict(line, offset=121, width=31)))
        accepted = tracker.update(dict(line, offset=119, width=30))

        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["offset"], 119)

    def test_band_tracker_rejects_large_offset_jump(self):
        tracker = GuidanceBandTracker(
            max_abs_offset=100,
            max_offset_jump=20,
            max_width_change=8,
            min_stable_frames=2,
        )

        self.assertIsNone(tracker.update({"mode": "bright_band", "offset": 10, "angle": 0, "width": 30}))
        self.assertIsNone(tracker.update({"mode": "bright_band", "offset": 55, "angle": 0, "width": 31}))
        accepted = tracker.update({"mode": "bright_band", "offset": 56, "angle": 0, "width": 31})

        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["offset"], 56)

    def test_band_tracker_rejects_large_width_change(self):
        tracker = GuidanceBandTracker(
            max_abs_offset=100,
            max_offset_jump=20,
            max_width_change=8,
            min_stable_frames=2,
        )

        self.assertIsNone(tracker.update({"mode": "bright_band", "offset": 10, "angle": 0, "width": 30}))
        self.assertIsNone(tracker.update({"mode": "bright_band", "offset": 11, "angle": 0, "width": 50}))
        accepted = tracker.update({"mode": "bright_band", "offset": 12, "angle": 0, "width": 51})

        self.assertIsNotNone(accepted)
        self.assertEqual(accepted["width"], 51)


if __name__ == "__main__":
    unittest.main()
