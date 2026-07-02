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
            "CORNER_IMU_ENTRY_RAMP_DEG",
            "CORNER_IMU_FORCE_MIN_SCALE",
            "CORNER_YAW_EXIT_EPSILON_DEG",
            "RIGHT_ANGLE_ABANDON_CONFIRM_SAMPLES",
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

    def test_pretrigger_abandons_only_on_sensor_pattern_not_timeout(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        self.assertIn("RightAngleDetector_AllWhite", track)
        self.assertNotIn("RIGHT_ANGLE_PRE_TIMEOUT_MS", track)
        self.assertNotIn("_TIMEOUT_MS", track)
        phase1_start = track.index("else if (is_right_angle == 0 && right_angle_phase == 1U)")
        phase2_start = track.index("else if (is_right_angle != 0 && right_angle_phase == 2U)")
        phase1 = track[phase1_start:phase2_start]
        # 预触发可以放弃，但只能挂在传感器图案判据(AbandonConfirmed)之后，不能是超时
        abandon_idx = phase1.index("RightAngleDetector_AbandonConfirmed(&right_angle_detector)")
        reset_idx = phase1.index("Track_Reset_Right_Angle_State();")
        self.assertGreater(reset_idx, abandon_idx)

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

    def test_corner_is_pure_imu_execution_with_no_gray_handover(self):
        track = (ROOT / "project_1.0 car" / "bsp" / "track.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        config = (ROOT / "project_1.0 car" / "bsp" / "track_config.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        check_start = track.index("void Track_Check_Right_Angle(void)")
        action_start = track.index("void Track_Action_Execute(void)")
        check_phase1_start = track.index(
            "else if (is_right_angle == 0 && right_angle_phase == 1U)",
            check_start,
        )
        check_phase2_start = track.index(
            "else if (is_right_angle != 0 && right_angle_phase == 2U)",
            check_start,
        )
        check_phase2_end = track.index("void Track_PID_Calc", check_phase2_start)
        check_phase1 = track[check_phase1_start:check_phase2_start]
        check_phase2 = track[check_phase2_start:check_phase2_end]
        action_corner_start = track.index(
            "is_right_angle != 0 && right_angle_phase == 2U) {",
            action_start,
        )
        action_corner_end = track.index("else {", action_corner_start)
        action_corner = track[action_corner_start:action_corner_end]
        # 灰度渐变接管已完全移除：不存在phase3、不存在GrayBlend辅助函数，
        # 直角执行阶段(check_phase2/action_corner)不出现任何灰度相关判据
        self.assertNotIn("right_angle_phase == 3U", track)
        self.assertNotIn("Track_Corner_GrayBlend", track)
        self.assertNotIn("gray_blend", check_phase2 + action_corner)
        self.assertNotIn("CORNER_GRAY_BLEND_START_DEG", config + track)
        self.assertNotIn("RightAngleDetector_AllWhite", check_phase2)
        self.assertNotIn("Grayscale_GetDigital", check_phase2)
        # 出弯判据是IMU剩余偏航误差收敛，不是灰度状态
        self.assertIn("#define CORNER_YAW_EXIT_EPSILON_DEG   3.0f", config)
        self.assertIn(
            "if (fabsf(yaw_error) > CORNER_YAW_EXIT_EPSILON_DEG) {", check_phase2
        )
        # 入口力道爬升包络仍然生效，直角期间不掺入灰度PID修正
        self.assertIn("Track_Corner_Imu_ForceScale(yaw_turned)", action_corner)
        self.assertIn("gyro_diff = f_clamp(gyro_diff, imu_diff_limit);", action_corner)
        self.assertIn("pid_correction = 0.0f;", action_corner)
        self.assertNotIn("CORNER_GYRO_DIFF_LIMIT_RPM", config + track)
        self.assertNotIn("pid_smooth", config + track)
        # 圈数计数移到出弯瞬间：预触发阶段不计数，避免被放弃分支取消的
        # 误触发污染圈数；只有真正收敛完成的弯才计数
        self.assertNotIn("Track_RegisterCornerComplete();", check_phase1)
        self.assertIn("Track_RegisterCornerComplete();", check_phase2)

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

    def test_eye_clears_stale_k230_packets_to_unstick_state_machine(self):
        config = (ROOT / "project_1.0 eye" / "bsp" / "vision_config.h").read_text(
            encoding="utf-8", errors="ignore"
        )
        vision = (ROOT / "project_1.0 eye" / "bsp" / "vision.c").read_text(
            encoding="utf-8", errors="ignore"
        )
        process = vision[vision.index("void Vision_Process(void)"):]
        self.assertIn("VISION_K230_PACKET_TIMEOUT_MS", config)
        self.assertIn("packet_stale", process)
        self.assertIn("Vision_TargetProcess(0, 0, false);", process)
        self.assertIn("Vision_LaserTrigger(false);", process)
        self.assertIn("Vision_GimbalPID_ClearIntegral();", process)
        self.assertNotIn("Tracking_Update", process)
        self.assertNotIn("VisionStrategy", process)
        self.assertNotIn("roi_state", process)

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
