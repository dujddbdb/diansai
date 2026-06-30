# 2026-06-20 - 补全灰度模拟量 EMA 滤波

## 日期
- 2026-06-20

## 涉及文件
- `bsp/grayscale.h` — 新增 EMA 宏 + Analog_raw[8] + analog_ema_init 字段
- `bsp/grayscale.c` — Grayscale_Task 实现 EMA 滤波, Grayscale_InitFirst 清零新字段

## 修改前状态
- `2026-06-19-gray-analog-ema-filter.md` 文档描述了 EMA 滤波设计，但代码中完全没有实现
- `Grayscale_Task()` 直接把 8 次均值写入 `Analog_value`，二值化用原始值 → 边界噪声导致 ir_raw 抖动

## 修改内容

### 1. grayscale.h — 新增宏和结构体字段
```c
// EMA 滤波配置
#define GRAY_ANALOG_EMA_ENABLE        1    // 1=启用, 0=关闭
#define GRAY_ANALOG_EMA_PREV_WEIGHT   7    // 旧值权重 (70%)
#define GRAY_ANALOG_EMA_NEW_WEIGHT    3    // 新值权重 (30%)
#define GRAY_ANALOG_EMA_TOTAL_WEIGHT  10   // 权重总和

// 结构体新增
unsigned short Analog_raw[8];       // 原始8次均值 (未经EMA)
unsigned char  analog_ema_init;     // EMA首帧标志
```

### 2. grayscale.c — Grayscale_Task 改造
- Step1: `Gray_ReadAllCh()` → `Analog_raw` (不再是 Analog_value)
- Step1.5: EMA 滤波 `Analog_raw → Analog_value`
  - 首帧直通: `Analog_value[i] = Analog_raw[i]` (避免从0爬升)
  - 后续: `Analog_value = (old*7 + new*3) / 10`
- Step2/3: 用 `Analog_value`(滤波后)做二值化和归一化

### 3. Grayscale_GetAnalog 修正
- 采集写入 `Analog_raw` 而非 `Analog_value`，避免覆盖 EMA 滤波结果

### 4. Grayscale_InitFirst
- 新增 `memset(s->Analog_raw, 0, ...)` 和 `s->analog_ema_init = 0`

## 参数调优
| 场景 | PREV_WEIGHT | NEW_WEIGHT |
|------|-------------|------------|
| 当前(默认) | 7 | 3 |
| 传感器噪声大, ir_raw 频繁跳变 | 8 | 2 |
| 入弯/压线反应迟钝 | 6 | 4 |

## 测试建议
- 串口观察 ir_raw 在直道时是否稳定（不再偶发跳变）
- 慢速过弯检查是否因滤波延迟导致响应变慢
- 如果响应变慢: 改 `NEW_WEIGHT` 4, `PREV_WEIGHT` 6
