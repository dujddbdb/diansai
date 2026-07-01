# 智能车项目代码注释规范修复 - 验证清单

## Car项目 - 主程序
- [ ] main.c 头文件引用处无大段冗余注释
- [ ] main.c 主函数各初始化步骤有逻辑块注释
- [ ] main.c 主循环各任务模块有清晰注释
- [ ] 所有注释为中文

## Car项目 - 头文件
- [ ] track.h 所有函数声明有功能、参数、返回值说明
- [ ] track.h 所有全局变量有用途注释
- [ ] track.h 结构体及成员有注释
- [ ] grayscale.h 所有元素有详细注释
- [ ] encoder.h 所有元素有详细注释
- [ ] tb6612.h 所有元素有详细注释
- [ ] bno080.h 所有元素有详细注释
- [ ] peripheral.h 所有元素有详细注释
- [ ] key.h 所有元素有详细注释
- [ ] stepper.h 所有元素有详细注释
- [ ] uart_k230.h 所有元素有详细注释
- [ ] right_angle_detector.h 所有元素有详细注释
- [ ] corner_profile.h 所有元素有详细注释
- [ ] uart/uart3.h 所有元素有详细注释
- [ ] uart/uart5.h 所有元素有详细注释
- [ ] oled/oled.h 所有元素有详细注释
- [ ] board/board.h 所有元素有详细注释
- [ ] module/stm32f4xx_it.h 所有元素有详细注释
- [ ] 所有头文件注释为中文

## Car项目 - 配置文件
- [ ] track_config.h 所有宏定义有中文注释
- [ ] PID参数有调大/调小效果说明
- [ ] 滤波系数有调大/调小效果说明
- [ ] 阈值参数有调大/调小效果说明
- [ ] 状态机参数有简单注释
- [ ] 所有注释为中文

## Car项目 - 源文件
- [ ] track.c 函数内部逻辑块有注释
- [ ] track.c 硬件初始化函数有逐行注释
- [ ] track.c 状态机各阶段有清晰注释
- [ ] track.c PID计算有逻辑块注释
- [ ] grayscale.c 函数内部有逻辑块注释
- [ ] grayscale.c 硬件初始化函数有逐行注释
- [ ] encoder.c 硬件初始化函数有逐行注释
- [ ] tb6612.c 硬件初始化函数有逐行注释
- [ ] bno080.c 硬件初始化函数有逐行注释
- [ ] peripheral.c 有适当注释
- [ ] key.c 有适当注释
- [ ] stepper.c 有适当注释
- [ ] uart_k230.c 有适当注释
- [ ] uart/uart3.c 硬件初始化有逐行注释
- [ ] uart/uart5.c 硬件初始化有逐行注释
- [ ] oled/oled.c 有适当注释
- [ ] board/board.c 有适当注释
- [ ] module/stm32f4xx_it.c 中断函数有注释
- [ ] 所有源文件注释为中文

## Eye项目 - 主程序
- [ ] main.c 头文件引用处无大段冗余注释
- [ ] main.c 主函数各初始化步骤有逻辑块注释
- [ ] main.c 主循环各任务模块有清晰注释
- [ ] 所有注释为中文

## Eye项目 - 头文件
- [ ] vision.h 所有元素有详细注释
- [ ] vision_strategy.h 所有元素有详细注释
- [ ] gimbal_pid.h 所有元素有详细注释
- [ ] grayscale.h 所有元素有详细注释
- [ ] encoder.h 所有元素有详细注释
- [ ] tb6612.h 所有元素有详细注释
- [ ] bno080.h 所有元素有详细注释
- [ ] peripheral.h 所有元素有详细注释
- [ ] key.h 所有元素有详细注释
- [ ] stepper.h 所有元素有详细注释
- [ ] uart_k230.h 所有元素有详细注释
- [ ] uart/uart3.h 所有元素有详细注释
- [ ] uart/uart5.h 所有元素有详细注释
- [ ] uart/car_corner_protocol.h 所有元素有详细注释
- [ ] oled/oled.h 所有元素有详细注释
- [ ] board/board.h 所有元素有详细注释
- [ ] module/stm32f4xx_it.h 所有元素有详细注释
- [ ] 所有头文件注释为中文

## Eye项目 - 配置文件
- [ ] vision_config.h 所有宏定义有中文注释
- [ ] PID参数有调大/调小效果说明
- [ ] 滤波系数有调大/调小效果说明
- [ ] 阈值参数有调大/调小效果说明
- [ ] 所有注释为中文

## Eye项目 - 源文件
- [ ] vision.c 函数内部逻辑块有注释
- [ ] vision_strategy.c 有逻辑块注释
- [ ] gimbal_pid.c 有逻辑块注释
- [ ] grayscale.c 硬件初始化有逐行注释
- [ ] encoder.c 硬件初始化有逐行注释
- [ ] tb6612.c 硬件初始化有逐行注释
- [ ] bno080.c 硬件初始化有逐行注释
- [ ] peripheral.c 有适当注释
- [ ] key.c 有适当注释
- [ ] stepper.c 有适当注释
- [ ] uart_k230.c 有适当注释
- [ ] uart/uart3.c 硬件初始化有逐行注释
- [ ] uart/uart5.c 硬件初始化有逐行注释
- [ ] oled/oled.c 有适当注释
- [ ] board/board.c 有适当注释
- [ ] module/stm32f4xx_it.c 中断函数有注释
- [ ] 所有源文件注释为中文

## K230项目 - Python文件
- [ ] main.py 所有函数有功能说明注释
- [ ] main.py 类和方法有详细注释
- [ ] main.py 可调参数有调大/调小效果说明
- [ ] main.py 核心算法逻辑块有注释
- [ ] main_kalman.py 有适当中文注释
- [ ] vision_math.py 有适当中文注释
- [ ] k230_axis_test.py 有适当中文注释
- [ ] 01test.py 有适当中文注释
- [ ] 所有注释为中文

## Git提交
- [ ] 所有修改文件已 git add
- [ ] 代码已 git commit 提交
- [ ] 代码已 git push 推送到远程仓库
