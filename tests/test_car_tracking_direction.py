import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).parents[1]


class CarTrackingDirectionTest(unittest.TestCase):
    def test_car_ir_weight_keeps_physical_right_positive_left_negative(self):
        source = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        match = re.search(r"raw_w\[8\]\s*=\s*\{([^}]+)\}", source)
        self.assertIsNotNone(match)
        weights = [int(x.strip()) for x in match.group(1).split(",")]
        self.assertEqual(weights[:4], [-7, -5, -3, -1])
        self.assertEqual(weights[4:], [1, 3, 5, 7])

    def test_right_angle_exit_still_uses_all_white_only(self):
        source = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        phase1_start = source.index("if (is_right_angle == 0 && right_angle_phase == 0U)")
        phase2_start = source.index("else if (is_right_angle != 0 && right_angle_phase == 2U)")
        phase1 = source[phase1_start:phase2_start]
        self.assertIn("RightAngleDetector_AllWhite", phase1)
        self.assertIn("RightAngleDetector_WhiteConfirmed", phase1)
        self.assertNotIn("RightAngleDetector_WhiteEnough", phase1)
        self.assertNotIn("gray_corner_diff_smooth", phase1.lower())
        self.assertIn("right_angle_initial_flag", phase1)
        self.assertIn("RightAngleDetector_ConfirmedFeature(&right_angle_detector)", phase1)
        self.assertNotIn("Grayscale_GetDigital(&gray_sensor)", phase1)
        self.assertNotIn("gyro_yaw_available", phase1)

    def test_right_angle_white_confirm_requires_eight_white_bits(self):
        detector = (ROOT / "project_1.0 car" / "bsp" / "right_angle_detector.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        update_start = detector.index("static inline uint8_t RightAngleDetector_Update")
        confirmed_start = detector.index("static inline uint8_t RightAngleDetector_ConfirmedFeature")
        update_body = detector[update_start:confirmed_start]
        self.assertIn("RightAngleDetector_AllWhite(detector->filtered)", update_body)
        self.assertNotIn("RightAngleDetector_WhiteEnough(detector->filtered)", update_body)


if __name__ == "__main__":
    unittest.main()
