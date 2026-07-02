import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).parents[1]


def read_text(*parts):
    return ROOT.joinpath(*parts).read_text(encoding="utf-8", errors="ignore")


def macro_float(config, name):
    match = re.search(rf"(?m)^#define\s+{name}\s+([0-9.]+)f\b", config)
    if not match:
        raise AssertionError(f"missing float macro {name}")
    return float(match.group(1))


class SmoothCornerK230PerfContracts(unittest.TestCase):
    def test_car_key_control_consumes_one_press_event(self):
        main_c = read_text("project_1.0 car", "app", "main.c")
        start = main_c.index("static void Car_KeyControlTick(void)")
        end = main_c.index("int main(void)", start)
        body = main_c[start:end]

        self.assertIn("uint8_t key = Key_GetTriggered();", body)
        self.assertNotIn("Key_GetState", body)
        self.assertIn("key == 2U", body)
        self.assertIn("Track_TargetLapAdd();", body)

    def test_corner_profile_keeps_constant_speed_and_smooth_windows(self):
        config = read_text("project_1.0 car", "bsp", "track_config.h")
        track = read_text("project_1.0 car", "bsp", "track.c")

        self.assertEqual(macro_float(config, "CORNER_IMU_MID_DEG"), 45.0)
        self.assertEqual(macro_float(config, "CORNER_GRAY_BLEND_START_DEG"), 75.0)
        self.assertEqual(macro_float(config, "CORNER_IMU_EXIT_DEG"), 85.0)
        for removed in (
            "CORNER_PID_SMOOTH_ALPHA",
            "CORNER_GYRO_DIFF_LIMIT_RPM",
            "CORNER_BASE_SLEW_DOWN_RPM_PER_MS",
            "CORNER_BASE_SLEW_UP_RPM_PER_MS",
            "WHEEL_COMMAND_SLEW_RPM_PER_MS",
        ):
            self.assertNotIn(removed, config + track)
        self.assertNotIn("CornerProfile_Slew(", track)

    def test_corner_completion_clears_debounced_detector_state(self):
        track = read_text("project_1.0 car", "bsp", "track.c")
        completion = track[track.index("Track_RegisterCornerComplete") - 900:]

        self.assertIn("RightAngleDetector_Init(&right_angle_detector);", completion)
        self.assertIn("right_angle_filtered_bits = 0xFFU;", completion)

    def test_k230_defaults_to_headless_grayscale_fast_path(self):
        main_py = read_text("k230", "main.py")

        self.assertRegex(main_py, r"(?m)^HEADLESS\s*=\s*True\b")
        self.assertRegex(main_py, r"(?m)^SHOW_TO_IDE\s*=\s*False\b")
        self.assertRegex(main_py, r"(?m)^SENSOR_PIXFORMAT\s*=\s*\"GRAYSCALE\"")
        self.assertIn("FULL_CHN = CAM_CHN_ID_0", main_py)
        self.assertIn("DETECT_CHN = CAM_CHN_ID_1", main_py)
        self.assertIn("sensor.set_framesize(width=CAM_W, height=CAM_H, chn=FULL_CHN)", main_py)
        self.assertIn("chn=DETECT_CHN, crop=CROP_ROI", main_py)
        self.assertIn("sensor.snapshot(chn=detect_chn)", main_py)
        self.assertIn("CV_LITE_FALLBACK_PERIOD = 12", main_py)
        self.assertIn("FIND_RECTS_FALLBACK_PERIOD = 30", main_py)
        self.assertIn("BLOB_X_STRIDE = 3", main_py)
        self.assertIn("cands = self._blob_rects(gray, roi)", main_py)
        self.assertLess(main_py.index("cands = self._blob_rects(gray, roi)"),
                        main_py.index("cands = self._cv_lite_rects(gray, roi)"))
        self.assertIn("self.frame_id % FIND_RECTS_FALLBACK_PERIOD", main_py)


if __name__ == "__main__":
    unittest.main()
