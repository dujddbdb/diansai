# BNO08x读包期间关闭控制定时器中断

**日期**: 2026-06-19

## 涉及文件
- `bsp/bno080.c`
- `bsp/bno080.h`

## 修改前状态
- 直角执行阶段只依赖陀螺仪 yaw 变化退出。
- 实车日志中进入 `ph1` 后 `yaw` 和 `rt` 长时间不更新，`st` 持续累加，表现为 BNO08x 数据在直角转弯时冻结。
- BNO08x I2C 读包过程中可能被 TIM11 控制环或 TIM7 速度环中断打断，导致 BNO08x SHTP/I2C 时序异常。

## 修改内容和原因
- 新增 `BNO080_TimerIrq_Lock()` / `BNO080_TimerIrq_Unlock()`，在 `bno080_update()` 真正读取 SHTP 包前停止 TIM11/TIM7 定时器，读完或读取失败后立即恢复。
- 不关闭全局中断，保留 SysTick/HAL_GetTick 超时能力；只让 TIM11/TIM7 在 BNO08x I2C 读包期间不进入中断，防止打断 SHTP/I2C 时序。
- 保持直角退出逻辑为纯陀螺仪 yaw 判断，没有加入灰度退出或超时退出。
- 保留 BNO08x 调试计数变量声明，便于后续继续定位 EXTI/I2C/解析层是否仍停顿。

## 测试建议
- 下载后测试直角，观察 `ph1` 时 `yaw` 是否连续变化、`st` 是否不再持续上升。
- 若仍冻结，继续观察串口 `ex/up/pk/rot/gf`：若 `up` 增加但 `pk/rot` 不增，重点查 I2C/INTN；若 `rot/gf` 增加但 yaw 不变，重点查四元数解析或 BNO08x 融合输出。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。

