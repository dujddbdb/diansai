# TIM10独占读取陀螺仪并从TIM11移除消费

**日期**: 2026-06-19

## 涉及文件
- `app/main.c`
- `bsp/bno080.c`
- `bsp/track.c`
- `bsp/track_config.h`

## 修改前状态
- BNO080 初始化成功后启动 TIM10，同时 TIM11 中仍调用 `Track_Gyro_Update()`。
- `Track_Gyro_Update()` 虽然不直接执行 I2C 读包，但会消费 BNO080 新数据并计算 `yaw/rt`，不应放在 1kHz 主控制环里。
- 前一版为隔离抖动临时加入了 `GYRO_RUNTIME_POLL_ENABLE`，会导致 TIM10 也被关闭，架构方向不正确。

## 修改内容和原因
- 删除 `GYRO_RUNTIME_POLL_ENABLE` 临时开关。
- `app/main.c` 中恢复 BNO080 初始化成功后直接启动 `BNO080_TIM10_Init()`，再启动 `Track_TIM11_Init()`。
- `bsp/track.c` 的 TIM11 主控制环不再调用 `Track_Gyro_Update()`，TIM11 只做灰度转换、直角状态、误差 PID 和动作输出。
- `bsp/bno080.c` 引入 `track.h`，在 TIM10 中断 `while (bno080_update()) {}` 排空数据后调用 `Track_Gyro_Update()`，由 TIM10 独占陀螺仪数据读取/消费链路。

## 测试建议
- 下载后观察串口：`gf` 应继续递增，`yaw/rt` 应继续刷新。
- `ph/t` 仍应保持 0，因为当前 `RIGHT_ANGLE_ENABLE=0`。
- 如果仍疯狂抖，重点看串口 `err/PID` 是否继续在 `+50000/-50000` 饱和跳变；若是，则问题在灰度数字量/误差链路或串口打印影响采样，不是陀螺仪 PID。
- 若 `yaw/rt` 不刷新，检查 TIM10 中断是否进入、BNO080 INTN 是否持续触发。

## 编译验证
- Keil MDK 构建通过：`0 Error(s), 0 Warning(s)`。
