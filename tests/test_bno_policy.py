import pathlib
import re
import unittest


SOURCE = (pathlib.Path(__file__).parents[1] / "project_1.0 car" / "bsp" / "bno080.c")
EYE_SOURCE = (pathlib.Path(__file__).parents[1] / "project_1.0 eye" / "bsp" / "bno080.c")
CAR_TRACK = (pathlib.Path(__file__).parents[1] / "project_1.0 car" / "bsp" / "track.c")
EYE_MAIN = (pathlib.Path(__file__).parents[1] / "project_1.0 eye" / "app" / "main.c")


class BnoPolicyTest(unittest.TestCase):
    def test_runtime_i2c_timeout_is_bounded(self):
        text = SOURCE.read_text(encoding="utf-8", errors="replace")
        value = int(re.search(r"#define\s+I2C_TRANSFER_TIMEOUT_MS\s+(\d+)U", text).group(1))
        self.assertLessEqual(value, 5)

    def test_autonomous_reset_packet_is_recognized(self):
        text = SOURCE.read_text(encoding="utf-8", errors="replace")
        self.assertIn("CHANNEL_EXECUTABLE", text[text.index("bno080_handle_packet"):])
        self.assertIn("BNO080_EXECUTABLE_RESET_COMPLETE", text)

    def test_runtime_read_does_not_mask_control_timers(self):
        text = SOURCE.read_text(encoding="utf-8", errors="replace")
        update = text[text.index("uint8_t bno080_update(void)"):text.index("uint8_t bno080_data_available(void)")]
        self.assertNotIn("BNO080_TimerIrq_Lock();", update)
        self.assertNotIn("BNO080_TimerIrq_Unlock();", update)

    def test_eye_imu_uses_same_bounded_reset_policy(self):
        text = EYE_SOURCE.read_text(encoding="utf-8", errors="replace")
        value = int(re.search(r"#define\s+I2C_TRANSFER_TIMEOUT_MS\s+(\d+)U", text).group(1))
        self.assertLessEqual(value, 5)
        self.assertIn("BNO080_EXECUTABLE_RESET_COMPLETE", text)

    def test_both_imus_require_first_data_before_init_success(self):
        for source in (SOURCE, EYE_SOURCE):
            text = source.read_text(encoding="utf-8", errors="replace")
            init = text[text.index("uint8_t bno080_init(void)"):text.index("uint8_t bno080_update(void)")]
            self.assertIn("if (g_bno080.new_data)", init)
            self.assertIn("g_bno080.new_data = 0", init)
            self.assertRegex(init, r"return\s+0;\s*}")

    def test_power_on_waits_until_both_imus_have_runtime_frame(self):
        track = CAR_TRACK.read_text(encoding="utf-8", errors="replace")
        car_init = track[track.index("void Track_Init(void)"):]
        self.assertIn("while (!gyro_yaw_available)", car_init)
        self.assertIn("gyro_poll_request = 1U;", car_init)
        self.assertIn("Track_Main_Gyro();", car_init)
        self.assertIn("Track_Gyro_Update();", car_init)

        eye_main = EYE_MAIN.read_text(encoding="utf-8", errors="replace")
        wait = eye_main[eye_main.index("static uint8_t Eye_WaitForIMUData"):
                        eye_main.index("int main(void)")]
        self.assertIn("while (1)", wait)
        self.assertIn("BNO080_I2C_Init();", wait)
        self.assertIn("bno080_init()", wait)
        self.assertIn("bno080_update()", wait)
        self.assertIn("bno080_data_available()", wait)
        self.assertIn("bno080_get_euler", wait)


if __name__ == "__main__":
    unittest.main()
