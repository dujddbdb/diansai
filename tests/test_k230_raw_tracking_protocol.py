from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_text(*parts):
    return ROOT.joinpath(*parts).read_text(encoding="utf-8", errors="ignore")


def test_k230_sends_four_raw_tracking_bytes():
    main_py = read_text("k230", "main.py")

    assert "return bytes((err_y >> 8, err_y & 0xFF," in main_py
    assert "err_z >> 8, err_z & 0xFF))" in main_py
    assert "bytes((0xAA, 0x04" not in main_py
    assert "ERR_INVALID = 0x7FFF" in main_py
    assert "def control_error(cx, cy):" in main_py
    assert "err_y, err_z = control_error(cx, cy)" in main_py
    assert "view = img.copy(roi=crop_roi)" in main_py


def test_eye_parser_only_accepts_raw_tracking_error():
    header = read_text("project_1.0 eye", "bsp", "uart_k230.h")
    source = read_text("project_1.0 eye", "bsp", "uart_k230.c")
    vision = read_text("project_1.0 eye", "bsp", "vision.c")
    vision_h = read_text("project_1.0 eye", "bsp", "vision.h")

    assert "#define K230_TRACK_RAW_LEN 4U" in header
    assert "#define K230_TRACK_INVALID 0x7FFF" in header
    assert "int16_t err_y;" in header
    assert "int16_t err_z;" in header
    assert "if (k230_rx_count == K230_TRACK_RAW_LEN)" in source
    assert "k230_parsed.err_y = K230_ReadI16BE(k230_rx_buf[0], k230_rx_buf[1]);" in source
    assert "k230_parsed.err_z = K230_ReadI16BE(k230_rx_buf[2], k230_rx_buf[3]);" in source
    assert "K230_TRACK_INVALID" in source
    assert "K230_FRAME_HEADER" not in header + source
    assert "K230_CMD_" not in header + source
    assert "K230_SendSensorData" not in header + source + vision
    assert "track_fps" not in header + source + vision
    assert "heartbeat" not in header + source + vision
    assert "VISION_IMG_CENTER" not in vision_h
    assert "VISION_LASER_ORIGIN" not in vision_h
    assert "Vision_PredictError" not in vision
