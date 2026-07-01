from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def read_text(path):
    local_path = ROOT / path
    if local_path.exists():
        return local_path.read_text(encoding="utf-8")
    return (ROOT.parent / path).read_text(encoding="utf-8")


def assert_pattern(text, pattern, message):
    if re.search(pattern, text, re.MULTILINE) is None:
        raise AssertionError(message)


def test_lcd_is_enabled_without_ide_stream():
    main_py = read_text("k230/main.py")
    assert_pattern(main_py, r"^HEADLESS\s*=\s*False\b", "LCD mode should be enabled")
    assert_pattern(main_py, r"^SHOW_TO_IDE\s*=\s*False\b", "IDE streaming should stay disabled")


def test_pid_gain_schedule_is_gradual():
    config = read_text("bsp/vision_config.h")
    header = read_text("bsp/gimbal_pid.h")
    source = read_text("bsp/gimbal_pid.c")

    for name in (
        "PID_ERROR_BLEND_START",
        "PID_ERROR_BLEND_END",
        "PID_GAIN_SLEW_FACTOR",
    ):
        if name not in config:
            raise AssertionError("%s should be declared in vision_config.h" % name)

    for name in (
        "kp_runtime",
        "ki_runtime",
        "kd_runtime",
    ):
        if name not in header:
            raise AssertionError("%s should be declared in gimbal_pid.h" % name)

    for name in (
        "GimbalPID_BlendFactor",
        "GimbalPID_UpdateRuntimeGains",
        "pid->kp_runtime",
        "pid->ki_runtime",
        "pid->kd_runtime",
    ):
        if name not in source:
            raise AssertionError("%s should be used in gimbal_pid.c" % name)


def test_right_angle_imu_compensation_is_enabled():
    config = read_text("bsp/vision_config.h")
    header = read_text("bsp/gimbal_pid.h")
    source = read_text("bsp/gimbal_pid.c")
    vision_c = read_text("bsp/vision.c")

    for name in (
        "GIMBAL_PITCH_DELTA_THRESHOLD",
    ):
        if name not in config:
            raise AssertionError("%s should be declared in vision_config.h" % name)

    for name in (
        "pitch_delta",
        "GimbalDualPID_SetIMUDelta",
    ):
        if name not in header:
            raise AssertionError("%s should be declared in gimbal_pid.h" % name)

    for name in (
        "pitch_compensation_angle",
        "output_y += pitch_compensation_angle",
        "dual_pid->pitch_delta = 0.0f",
    ):
        if name not in source:
            raise AssertionError("%s should be used in gimbal_pid.c" % name)

    for name in (
        "last_pitch_deg",
        "pitch_delta",
        "GimbalDualPID_SetIMUDelta",
    ):
        if name not in vision_c:
            raise AssertionError("%s should be used in bsp/vision.c" % name)


def test_imu_feedforward_runs_at_1khz_tick():
    config = read_text("bsp/vision_config.h")
    header = read_text("bsp/gimbal_pid.h")
    source = read_text("bsp/gimbal_pid.c")
    vision_c = read_text("bsp/vision.c")
    main_c = read_text("app/main.c")
    bno_c = read_text("bsp/bno080.c")

    for name in (
        "VISION_IMU_FEEDFORWARD_PERIOD_MS",
        "VISION_IMU_DRAIN_MAX_PACKETS",
        "VISION_IMU_FEEDFORWARD_ALWAYS_ON",
        "VISION_CONTROL_PERIOD_MS",
        "VISION_IMU_KALMAN_ANGLE_Q",
        "VISION_IMU_KALMAN_RATE_Q",
        "VISION_IMU_KALMAN_BIAS_Q",
        "VISION_IMU_KALMAN_MEAS_R",
        "VISION_IMU_KALMAN_RATE_DECAY",
        "VISION_IMU_BIAS_LIMIT_DEG",
        "VISION_IMU_RATE_LIMIT_DPS",
    ):
        if name not in config:
            raise AssertionError("%s should be declared in vision_config.h" % name)

    if "#define BNO080_REPORT_INTERVAL_MS    1U" not in bno_c:
        raise AssertionError("EYE BNO080 should request the fastest report interval")
    if "GimbalDualPID_UpdateFeedforward" not in header + source:
        raise AssertionError("IMU feedforward should have a PID-state-safe API")
    if "Vision_ReadLatestIMUFrame" not in vision_c:
        raise AssertionError("IMU tick should drain available BNO080 frames")
    if "VISION_IMU_DRAIN_MAX_PACKETS" not in vision_c:
        raise AssertionError("IMU drain loop should be bounded")
    if "predicted_measurement = kf->angle + kf->bias" not in vision_c:
        raise AssertionError("IMU Kalman should estimate drift/bias")
    if "VISION_IMU_KALMAN_BIAS_Q" not in vision_c:
        raise AssertionError("IMU Kalman bias process noise should be used")
    if "GimbalDualPID_UpdateFeedforward(&s_gimbal_pid)" not in vision_c:
        raise AssertionError("1kHz tick should not run the visual PID path")
    if "Vision_GimbalIMUCompensationTick();" not in main_c:
        raise AssertionError("main loop should call 1kHz IMU feedforward wrapper")
    if "Eye_WaitForIMUData" not in main_c:
        raise AssertionError("EYE startup should wait for the first IMU frame")


if __name__ == "__main__":
    test_lcd_is_enabled_without_ide_stream()
    test_pid_gain_schedule_is_gradual()
    test_right_angle_imu_compensation_is_enabled()
    test_imu_feedforward_runs_at_1khz_tick()
