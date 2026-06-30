# 2026-06-22 - Track_Main_Debug 轮换分页打印

## 日期
- 2026-06-22

## 背景
- 原 `Track_Main_Debug(200)` 每 200ms 一次性打印灰度、PID、速度、直角、陀螺仪全部数据。
- 串口发送是逐字节阻塞式 `USART3_SendByte()`, 单条长日志会占用主循环时间, 影响灰度采样和 BNO08x 轮询。

## 涉及文件
- `bsp/track.c`
- `tools/test_track_logic.py`

## 修改内容
- `Track_Main_Debug()` 新增 `debug_page`, 每次调用只打印一个页面, 下次自动切换到下一页。
- 缓冲区从 `char buf[256]` 改为 `char buf[128]`, 降低栈占用。
- 页面含义:
  - `D0`: 灰度二值化、循迹误差、PID输出。
  - `D1`: 左右目标转速和实际转速。
  - `D2`: 基础速度斜坡、灰度直角差速、直角状态。
  - `D3`: 陀螺仪 yaw、原始 yaw、角速度、BNO08x 数据流诊断。

## 验证
- `python tools\test_track_logic.py`: 8 tests passed。
- Keil MDK 命令行构建: `0 Error(s), 0 Warning(s)`。
