import pathlib
import re
import unittest


SOURCE = (pathlib.Path(__file__).parents[1] / "project_1.0 car" / "bsp" / "bno080.c")
EYE_SOURCE = (pathlib.Path(__file__).parents[1] / "project_1.0 eye" / "bsp" / "bno080.c")


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


if __name__ == "__main__":
    unittest.main()
