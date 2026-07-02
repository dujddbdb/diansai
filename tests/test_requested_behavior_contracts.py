import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).parents[1]


def read_project_text(*parts):
    return (ROOT.joinpath(*parts)).read_text(encoding="utf-8", errors="ignore")


class RequestedBehaviorContracts(unittest.TestCase):
    def test_car_right_angle_has_no_global_detect_lock(self):
        track_c = read_project_text("project_1.0 car", "bsp", "track.c")
        track_h = read_project_text("project_1.0 car", "bsp", "track.h")

        self.assertNotIn("global_detect_locked", track_c)
        self.assertNotIn("global_detect_timer", track_c)
        self.assertNotIn("global_detected_locked", track_h)
        self.assertNotIn("global_detect_timer", track_h)

    def test_car_pretrigger_uses_strict_all_white_only(self):
        track_c = read_project_text("project_1.0 car", "bsp", "track.c")
        phase1_start = track_c.index("else if (is_right_angle == 0 && right_angle_phase == 1U)")
        phase2_start = track_c.index("else if (is_right_angle != 0 && right_angle_phase == 2U)")
        phase1 = track_c[phase1_start:phase2_start]

        self.assertIn("RightAngleDetector_AllWhite", phase1)
        self.assertIn("RightAngleDetector_WhiteConfirmed", phase1)
        self.assertNotIn("RightAngleDetector_WhiteEnough", phase1)
        self.assertNotIn("RIGHT_ANGLE_PRE_TIMEOUT_MS", phase1)

    def test_eye_disables_euler_closed_loop_and_uses_imu_delta(self):
        gimbal_h = read_project_text("project_1.0 eye", "bsp", "gimbal_pid.h")
        gimbal_c = read_project_text("project_1.0 eye", "bsp", "gimbal_pid.c")
        vision_c = read_project_text("project_1.0 eye", "bsp", "vision.c")

        self.assertIn("GimbalDualPID_SetIMUDelta", vision_c)
        self.assertNotIn("GIMBAL_EULER", gimbal_h + gimbal_c + vision_c)
        self.assertNotIn("SetEuler", gimbal_h + gimbal_c + vision_c)
        self.assertNotIn("GimbalDualPID_SetYawDelta(&s_gimbal_pid", vision_c)
        self.assertNotIn("GimbalDualPID_SetPitchDelta(&s_gimbal_pid", vision_c)

    def test_k230_crop_center_is_single_origin_calibration(self):
        main_py = read_project_text("k230", "main.py")

        self.assertRegex(main_py, r"(?m)^IMG_W\s*=\s*320\b")
        self.assertRegex(main_py, r"(?m)^IMG_H\s*=\s*240\b")
        self.assertRegex(main_py, r"(?m)^CAM_W\s*=\s*1920\b")
        self.assertRegex(main_py, r"(?m)^CAM_H\s*=\s*1080\b")
        self.assertRegex(main_py, r"(?m)^CROP_CENTER_X\s*=\s*CAM_W // 2\b")
        self.assertRegex(main_py, r"(?m)^CROP_CENTER_Y\s*=\s*CAM_H // 2 - 77\b")
        self.assertIn("CROP_X = max(0, min(CAM_W - IMG_W, CROP_CENTER_X - IMG_W // 2))", main_py)
        self.assertIn("CROP_Y = max(0, min(CAM_H - IMG_H, CROP_CENTER_Y - IMG_H // 2))", main_py)
        self.assertIn("CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)", main_py)
        self.assertIn("def detection_center_roi", main_py)
        self.assertIn("ORIGIN_X = IMG_W // 2", main_py)
        self.assertIn("ORIGIN_Y = IMG_H // 2", main_py)
        self.assertIn("return CROP_ROI", main_py)
        self.assertIn("center_penalty = (abs(cx - ORIGIN_X) +", main_py)
        self.assertIn("abs(cy - ORIGIN_Y)) * 0.01", main_py)
        self.assertIn("chn=DETECT_CHN, crop=CROP_ROI", main_py)
        self.assertIn("view = img.copy(roi=detection_center_roi())", main_py)
        self.assertNotIn("LASER_OFFSET_X", main_py)
        self.assertNotIn("LASER_OFFSET_Y", main_py)
        self.assertNotIn("find_laser", main_py.lower())
        self.assertNotIn("crop_x = LASER_BIG_X - IMG_W // 2", main_py)

    def test_k230_has_simple_kalman_variant(self):
        kalman_py = read_project_text("k230", "main_kalman.py")

        self.assertIn("class TargetKalman", kalman_py)
        self.assertIn("IMG_W = 320", kalman_py)
        self.assertIn("IMG_H = 240", kalman_py)
        self.assertIn("CAM_W = 1920", kalman_py)
        self.assertIn("CAM_H = 1080", kalman_py)
        self.assertIn("CROP_ROI = (CROP_X, CROP_Y, IMG_W, IMG_H)", kalman_py)
        self.assertIn("ORIGIN_X = IMG_W // 2", kalman_py)
        self.assertIn("ORIGIN_Y = IMG_H // 2", kalman_py)
        self.assertIn("return bytes((err_y >> 8, err_y & 0xFF,", kalman_py)
        self.assertIn("view = frame.copy(roi=detection_center_roi())", kalman_py)


if __name__ == "__main__":
    unittest.main()
