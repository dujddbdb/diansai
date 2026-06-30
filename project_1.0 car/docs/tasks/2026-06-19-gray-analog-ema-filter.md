# 灰度模拟量 EMA 滤波

**日期**: 2026-06-19

## 涉及文件
- `bsp/grayscale.h`
- `bsp/grayscale.c`

## 修改前状态
- `Gray_ReadAllCh()` 每路做 8 次 ADC 均值后直接写入 `Analog_value`。
- `Grayscale_Task()` 使用这组模拟量直接二值化并归一化。
- 用户希望参考常见 ADC 滤波方式，让“上一次占比多少、下一次占比多少”，得到最终使用的模拟量。

## 修改内容和原因
- 新增一阶 EMA 滤波：`filtered = prev * 70% + current * 30%`。
- 新增 `Analog_raw[8]` 保存每路 8 次均值后的原始采样。
- `Analog_value[8]` 改为滤波后的最终模拟量，继续供二值化、归一化和调试输出使用。
- 第一次采样时直接用当前值初始化滤波历史，避免上电初始值从 0 慢慢爬升。

## 参数位置
- `GRAY_ANALOG_EMA_ENABLE`
- `GRAY_ANALOG_EMA_PREV_WEIGHT`
- `GRAY_ANALOG_EMA_NEW_WEIGHT`
- `GRAY_ANALOG_EMA_TOTAL_WEIGHT`

当前比例为旧值 `7`、新值 `3`。如果车对黑线响应变慢，可改成 `6/4` 或 `5/5`；如果传感器噪声导致二值跳变，再改成 `8/2`。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 1 Warning(s)`。
- 剩余警告为 `bno080.c` 文件末尾无换行，和本次灰度滤波无关。

## 测试建议
- 先架空或低速观察串口二值位是否比之前少跳。
- 如果入弯、压线时反应明显变钝，将新采样占比从 `3` 提到 `4`。
- 如果直道偶发误判黑线，将旧值占比从 `7` 提到 `8`。
