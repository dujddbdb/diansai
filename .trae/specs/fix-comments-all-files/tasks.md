# 智能车项目代码注释规范修复 - 实施计划

## 项目总览
共三个项目需要处理：
1. **project_1.0 car/** - 智能车巡线控制系统
2. **project_1.0 eye/** - 视觉云台追踪系统
3. **k230/** - K230视觉检测Python脚本

---

## [ ] Task 1: Car项目 - 清理冗余头文件引用注释 + 主程序注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 清理 main.c 中头文件引用的大段冗余注释
  - 主函数内部各逻辑块添加清晰中文注释
  - 主循环各任务模块添加注释
- **Acceptance Criteria Addressed**: AC-2, AC-5, AC-6
- **Test Requirements**:
  - `human-judgement` TR-1.1: #include 处无大段冗余说明注释
  - `human-judgement` TR-1.2: 主函数各初始化步骤和主循环各模块有清晰逻辑块注释
  - `human-judgement` TR-1.3: 所有注释为中文
- **Notes**: 保持简洁，不要过度注释
- **涉及文件**: project_1.0 car/app/main.c

## [ ] Task 2: Car项目 - 头文件详细注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 所有头文件的函数声明添加详细功能说明、参数说明、返回值说明
  - 全局变量添加用途注释
  - 结构体和成员变量添加注释
  - 宏定义添加含义说明
- **Acceptance Criteria Addressed**: AC-1, AC-6
- **Test Requirements**:
  - `human-judgement` TR-2.1: 所有 extern 全局变量有清晰用途注释
  - `human-judgement` TR-2.2: 所有函数声明有功能说明、参数说明、返回值说明
  - `human-judgement` TR-2.3: 结构体及成员有注释
  - `human-judgement` TR-2.4: 所有注释为中文
- **Notes**: 头文件注释要详细，方便调用者理解
- **涉及文件**:
  - project_1.0 car/bsp/track.h
  - project_1.0 car/bsp/grayscale.h
  - project_1.0 car/bsp/encoder.h
  - project_1.0 car/bsp/tb6612.h
  - project_1.0 car/bsp/bno080.h
  - project_1.0 car/bsp/peripheral.h
  - project_1.0 car/bsp/key.h
  - project_1.0 car/bsp/stepper.h
  - project_1.0 car/bsp/uart_k230.h
  - project_1.0 car/bsp/right_angle_detector.h
  - project_1.0 car/bsp/corner_profile.h
  - project_1.0 car/bsp/uart/uart3.h
  - project_1.0 car/bsp/uart/uart5.h
  - project_1.0 car/bsp/oled/oled.h
  - project_1.0 car/board/board.h
  - project_1.0 car/module/stm32f4xx_it.h

## [ ] Task 3: Car项目 - 配置文件参数调优说明
- **Priority**: high
- **Depends On**: None
- **Description**:
  - track_config.h 中所有宏定义添加中文注释
  - PID参数、滤波系数、阈值等可调参数添加调大调小效果说明
  - 状态机相关参数简单注释
- **Acceptance Criteria Addressed**: AC-1, AC-4, AC-6
- **Test Requirements**:
  - `human-judgement` TR-3.1: 所有宏定义有注释说明含义
  - `human-judgement` TR-3.2: PID参数、滤波系数等可调参数说明调大/调小的影响
  - `human-judgement` TR-3.3: 所有注释为中文
- **Notes**: 调优说明要实用，帮助参数调试
- **涉及文件**: project_1.0 car/bsp/track_config.h

## [ ] Task 4: Car项目 - 源文件逻辑块+底层函数逐行注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 函数内部按逻辑块添加中文注释
  - 控制底层函数（硬件初始化、寄存器配置等）逐行简单注释
  - 状态机逻辑各阶段添加清晰注释
  - PID计算等核心算法添加逻辑块注释
- **Acceptance Criteria Addressed**: AC-2, AC-3, AC-6
- **Test Requirements**:
  - `human-judgement` TR-4.1: 每个函数内部逻辑块有注释
  - `human-judgement` TR-4.2: 硬件初始化类函数关键行有逐行注释
  - `human-judgement` TR-4.3: 状态机各阶段有清晰注释
  - `human-judgement` TR-4.4: 所有注释为中文
- **Notes**: 核心文件注释要详细
- **涉及文件**:
  - project_1.0 car/bsp/track.c
  - project_1.0 car/bsp/grayscale.c
  - project_1.0 car/bsp/encoder.c
  - project_1.0 car/bsp/tb6612.c
  - project_1.0 car/bsp/bno080.c
  - project_1.0 car/bsp/peripheral.c
  - project_1.0 car/bsp/key.c
  - project_1.0 car/bsp/stepper.c
  - project_1.0 car/bsp/uart_k230.c
  - project_1.0 car/bsp/uart/uart3.c
  - project_1.0 car/bsp/uart/uart5.c
  - project_1.0 car/bsp/oled/oled.c
  - project_1.0 car/board/board.c
  - project_1.0 car/module/stm32f4xx_it.c

## [ ] Task 5: Eye项目 - 清理冗余头文件引用注释 + 主程序注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 清理 main.c 中头文件引用的冗余注释
  - 主函数内部各逻辑块添加清晰中文注释
  - 主循环各任务模块添加注释
- **Acceptance Criteria Addressed**: AC-2, AC-5, AC-6
- **Test Requirements**:
  - `human-judgement` TR-5.1: #include 处无大段冗余说明注释
  - `human-judgement` TR-5.2: 主函数各初始化步骤和主循环各模块有清晰逻辑块注释
  - `human-judgement` TR-5.3: 所有注释为中文
- **涉及文件**: project_1.0 eye/app/main.c

## [ ] Task 6: Eye项目 - 头文件详细注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 所有头文件的函数声明添加详细功能说明、参数说明、返回值说明
  - 全局变量添加用途注释
  - 结构体和成员变量添加注释
  - 宏定义添加含义说明
- **Acceptance Criteria Addressed**: AC-1, AC-6
- **Test Requirements**:
  - `human-judgement` TR-6.1: 所有全局变量有清晰用途注释
  - `human-judgement` TR-6.2: 所有函数声明有功能说明、参数说明、返回值说明
  - `human-judgement` TR-6.3: 结构体及成员有注释
  - `human-judgement` TR-6.4: 所有注释为中文
- **涉及文件**:
  - project_1.0 eye/bsp/vision.h
  - project_1.0 eye/bsp/vision_strategy.h
  - project_1.0 eye/bsp/gimbal_pid.h
  - project_1.0 eye/bsp/grayscale.h
  - project_1.0 eye/bsp/encoder.h
  - project_1.0 eye/bsp/tb6612.h
  - project_1.0 eye/bsp/bno080.h
  - project_1.0 eye/bsp/peripheral.h
  - project_1.0 eye/bsp/key.h
  - project_1.0 eye/bsp/stepper.h
  - project_1.0 eye/bsp/uart_k230.h
  - project_1.0 eye/bsp/uart/uart3.h
  - project_1.0 eye/bsp/uart/uart5.h
  - project_1.0 eye/bsp/uart/car_corner_protocol.h
  - project_1.0 eye/bsp/oled/oled.h
  - project_1.0 eye/board/board.h
  - project_1.0 eye/module/stm32f4xx_it.h

## [ ] Task 7: Eye项目 - 配置文件参数调优说明
- **Priority**: high
- **Depends On**: None
- **Description**:
  - vision_config.h 中所有宏定义添加中文注释
  - PID参数、滤波系数、阈值等可调参数添加调大调小效果说明
- **Acceptance Criteria Addressed**: AC-1, AC-4, AC-6
- **Test Requirements**:
  - `human-judgement` TR-7.1: 所有宏定义有注释说明含义
  - `human-judgement` TR-7.2: PID参数、滤波系数等可调参数说明调大/调小的影响
  - `human-judgement` TR-7.3: 所有注释为中文
- **涉及文件**: project_1.0 eye/bsp/vision_config.h

## [ ] Task 8: Eye项目 - 源文件逻辑块+底层函数逐行注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 函数内部按逻辑块添加中文注释
  - 控制底层函数逐行简单注释
  - 视觉处理、PID控制等核心算法添加逻辑块注释
- **Acceptance Criteria Addressed**: AC-2, AC-3, AC-6
- **Test Requirements**:
  - `human-judgement` TR-8.1: 每个函数内部逻辑块有注释
  - `human-judgement` TR-8.2: 硬件初始化类函数关键行有逐行注释
  - `human-judgement` TR-8.3: 所有注释为中文
- **涉及文件**:
  - project_1.0 eye/bsp/vision.c
  - project_1.0 eye/bsp/vision_strategy.c
  - project_1.0 eye/bsp/gimbal_pid.c
  - project_1.0 eye/bsp/grayscale.c
  - project_1.0 eye/bsp/encoder.c
  - project_1.0 eye/bsp/tb6612.c
  - project_1.0 eye/bsp/bno080.c
  - project_1.0 eye/bsp/peripheral.c
  - project_1.0 eye/bsp/key.c
  - project_1.0 eye/bsp/stepper.c
  - project_1.0 eye/bsp/uart_k230.c
  - project_1.0 eye/bsp/uart/uart3.c
  - project_1.0 eye/bsp/uart/uart5.c
  - project_1.0 eye/bsp/oled/oled.c
  - project_1.0 eye/board/board.c
  - project_1.0 eye/module/stm32f4xx_it.c

## [ ] Task 9: K230项目 - Python文件注释
- **Priority**: high
- **Depends On**: None
- **Description**:
  - 所有Python文件的函数添加中文注释（功能、参数、返回值）
  - 类和方法添加详细注释
  - 可调参数区添加调优说明（调大/调小效果）
  - 核心算法逻辑块添加注释
  - 清理英文注释，替换为中文
- **Acceptance Criteria Addressed**: AC-1, AC-2, AC-4, AC-6
- **Test Requirements**:
  - `human-judgement` TR-9.1: 所有函数有功能说明注释
  - `human-judgement` TR-9.2: 类和方法有详细注释
  - `human-judgement` TR-9.3: 可调参数有调大/调小效果说明
  - `human-judgement` TR-9.4: 核心算法逻辑块有注释
  - `human-judgement` TR-9.5: 所有注释为中文
- **涉及文件**:
  - k230/main.py
  - k230/main_kalman.py
  - k230/vision_math.py
  - k230/k230_axis_test.py
  - k230/01test.py

## [ ] Task 10: Git 提交和推送
- **Priority**: high
- **Depends On**: Task 1~9
- **Description**:
  - 确认所有文件修改完成
  - 使用 git add 添加所有修改文件
  - 使用 git commit 提交，提交信息清晰
  - 使用 git push 推送到远程仓库
- **Acceptance Criteria Addressed**: AC-9
- **Test Requirements**:
  - `programmatic` TR-10.1: git status 显示所有修改已提交
  - `programmatic` TR-10.2: git log 显示最新提交
  - `programmatic` TR-10.3: git push 成功
- **Notes**: 提交信息用中文，说明修改内容
