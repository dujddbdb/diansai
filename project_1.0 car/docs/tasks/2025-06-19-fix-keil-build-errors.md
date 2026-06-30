# 修复 Keil 编译错误

**日期**: 2025-06-19

## 涉及文件
- `bsp/track.c` — 添加 volatile 关键字
- `app/main.c` — 修正 oled.h 包含路径
- `project/MDK(V5)/Project.uvprojx` — 添加 oled.c/oled.h 到项目

## 修改前状态
Keil V5.39 编译出现 3 个编译错误 + 6 个链接错误，无法生成目标文件。

## 错误与修复

### 错误1: volatile 不匹配 (track.c:81-82)
- **错误**: `#147: declaration is incompatible with "volatile uint8_t global_detect_locked"`
- **原因**: `track.h` 声明为 `extern volatile`，但 `track.c` 定义时缺少 `volatile`
- **修复**: 两行添加 `volatile` 关键字

### 错误2: oled.h 找不到 (main.c:16)
- **错误**: `#5: cannot open source input file "oled.h"`
- **原因**: `oled.h` 在 `bsp/oled/` 子目录，不在 include path 根目录
- **修复**: `#include "oled.h"` → `#include "oled/oled.h"` (利用已有的 `..\..\bsp` include path)

### 错误3: OLED 函数未定义 (链接阶段)
- **错误**: `L6218E: Undefined symbol OLED_Clear/OLED_Init/...` (6个)
- **原因**: `oled.c` 未被加入 Keil 项目文件
- **修复**: 在 `Project.uvprojx` 的 BSP 组中添加 `oled.c` 和 `oled.h`

## 编译结果
```
Program Size: Code=9514 RO-data=8122 RW-data=228 ZI-data=2684
0 Error(s), 0 Warning(s)
```

## 测试建议
- 下载到 STM32F407 确认 OLED 正常显示
- 确认灰度循迹/直角检测功能正常（global_detect_locked 可用）
