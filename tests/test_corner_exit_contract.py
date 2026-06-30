import pathlib
import unittest


ROOT = pathlib.Path(__file__).parents[1]


class CornerExitContractTest(unittest.TestCase):
    def test_track_config_keeps_key_macros_active(self):
        config_path = ROOT / "project_1.0 car" / "bsp" / "track_config.h"
        lines = config_path.read_text(encoding="utf-8", errors="ignore").splitlines()
        active_macros = set()
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("#define "):
                active_macros.add(stripped.split()[1])

        for name in (
            "KP_NORMAL",
            "ERR_FILTER_ALPHA",
            "INTEGRAL_LIMIT",
            "KD_GYRO_STRAIGHT",
            "KP_CORNER_YAW",
            "CORNER_MAX_RPM",
            "CORNER_ENTRY_BLEND_PROGRESS",
            "CORNER_GRAY_BLEND_START_DEG",
            "CORNER_IMU_EXIT_DEG",
        ):
            self.assertIn(name, active_macros)

    def test_gyro_turn_has_no_hard_timeout_exit(self):
        config = (ROOT / "project_1.0 car" / "bsp" / "track_config.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertNotIn("CORNER_TURN_TIMEOUT_MS", config)
        self.assertNotIn("right_angle_turn_timer >= CORNER_TURN_TIMEOUT_MS", track)
        self.assertNotIn("CORNER_TURN_TIMEOUT_MS) {", track)
        self.assertNotIn("CORNER_TURN_TIMEOUT_MS", track)

    def test_gyro_init_blocks_until_first_frame_without_runtime_recovery(self):
        config = (ROOT / "project_1.0 car" / "bsp" / "track_config.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertNotIn("GYRO_RECOVER_STALE_MS", config)
        self.assertNotIn("GYRO_CORNER_RECOVER_STALE_MS", config)
        self.assertNotIn("BNO080_INIT_RETRY_MAX", config)
        main_gyro_start = track.index("void Track_Main_Gyro(void)")
        main_gyro_end = track.index("void Track_Main_Debug", main_gyro_start)
        main_gyro = track[main_gyro_start:main_gyro_end]
        self.assertNotIn("Track_Gyro_Recover", main_gyro)
        self.assertNotIn("Track_Gyro_Recover", track)
        init_start = track.index("void Track_Init(void)")
        init_body = track[init_start:]
        self.assertIn("while (!gyro_yaw_available)", init_body)
        self.assertIn("bno080_ok = bno080_init();", init_body)

    def test_led_does_not_toggle_forever(self):
        car_main = (ROOT / "project_1.0 car" / "app" / "main.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        eye_main = (ROOT / "project_1.0 eye" / "app" / "main.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertNotIn("ExtLED_Toggle();", car_main)
        self.assertNotIn("ExtLED_Toggle();", eye_main)

    def test_serial_debug_output_stays_in_track_main_debug(self):
        car_root = ROOT / "project_1.0 car"
        offenders = []
        for path in car_root.rglob("*.c"):
            if "Objects" in path.parts or "Listings" in path.parts:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if path.name == "track.c":
                allowed = text[text.index("void Track_Main_Debug(uint32_t time)") :]
                before = text[: text.index("void Track_Main_Debug(uint32_t time)")]
                if "USART3_SendByte(" in before or "USART3_SendString(" in before:
                    offenders.append(str(path))
                if "USART3_SendByte(" not in allowed:
                    offenders.append(str(path))
            elif path.name != "uart3.c":
                if "USART3_SendByte(" in text or "USART3_SendString(" in text:
                    offenders.append(str(path))
        self.assertEqual([], offenders)

    def test_pretrigger_waits_for_strict_white_without_timeout_cancel(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("RightAngleDetector_AllWhite", track)
        self.assertNotIn("RIGHT_ANGLE_PRE_TIMEOUT_MS", track)

    def test_armed_gyro_never_switches_to_fallback_action(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertNotIn("if (!right_angle_gyro_armed)", track)
        self.assertNotIn("black_seen", track)

    def test_right_angle_type_keeps_direction_mapping(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("is_right_angle == 1", track)
        self.assertIn("gyro_corner_target_yaw = gyro_corner_start_yaw - CORNER_YAW_TARGET;", track)
        self.assertIn("gyro_corner_target_yaw = gyro_corner_start_yaw + CORNER_YAW_TARGET;", track)

    def test_corner_exit_is_imu_angle_based_with_gray_handover(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        check_start = track.index("void Track_Check_Right_Angle(void)")
        action_start = track.index("void Track_Action_Execute(void)")
        check_phase3_start = track.index(
            "else if (is_right_angle != 0 && right_angle_phase == 3)",
            check_start,
        )
        check_phase3_end = track.index("void Track_PID_Calc", check_phase3_start)
        check_phase3 = track[check_phase3_start:check_phase3_end]
        action_corner_start = track.index(
            "(right_angle_phase == 2U || right_angle_phase == 3U)",
            action_start,
        )
        action_corner_end = track.index("else {", action_corner_start)
        action_corner = track[action_corner_start:action_corner_end]
        self.assertNotIn("gray_corner_diff_smooth", check_phase3 + action_corner)
        self.assertNotIn("RightAngleDetector_AllWhite", check_phase3)
        self.assertNotIn("right_angle_exit_gray_hits", check_phase3 + action_corner)
        self.assertIn("CORNER_IMU_EXIT_DEG", check_phase3)
        self.assertIn("CORNER_GRAY_BLEND_START_DEG", action_corner)
        self.assertIn("gyro_diff *= entry * (1.0f - gray_blend);", action_corner)
        self.assertIn("pid_correction = gray_blend * pid_smooth;", action_corner)

    def test_eye_uses_k230_reported_error_directly(self):
        eye_main = (ROOT / "project_1.0 eye" / "app" / "main.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        vision = (ROOT / "project_1.0 eye" / "bsp" / "vision.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        gimbal_header = (ROOT / "project_1.0 eye" / "bsp" / "gimbal_pid.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("Vision_Process", eye_main)
        self.assertIn("GimbalDualPID_SetIMUDelta(&s_gimbal_pid", vision)
        self.assertNotIn("GIMBAL_EULER", gimbal_header + vision)
        self.assertNotIn("SetEuler", gimbal_header + vision)
        self.assertNotIn("GimbalDualPID_SetYawDelta(&s_gimbal_pid", vision)
        self.assertNotIn("GimbalDualPID_SetPitchDelta(&s_gimbal_pid", vision)
        self.assertNotIn("Vision_GimbalPID_Update", eye_main)

    def test_eye_imu_compensation_only_during_corner_execution(self):
        uart5 = (ROOT / "project_1.0 eye" / "bsp" / "uart" / "uart5.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        phase1 = uart5[uart5.index("case 1U:"):uart5.index("case 2U:")]
        phase2 = uart5[uart5.index("case 2U:"):uart5.index("case 3U:")]
        phase3 = uart5[uart5.index("case 3U:"):uart5.index("default:")]
        self.assertIn("blend = 0.0f;", phase1)
        self.assertIn("blend = 1.0f;", phase2)
        self.assertIn("blend = 0.55f;", phase3)


if __name__ == "__main__":
    unittest.main()
