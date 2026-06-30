# 2026-06-22 - 修复直角偶发不进入执行

## 日期
- 2026-06-22

## 涉及文件
- `bsp/track.c`
- `bsp/track.h`
- `bsp/track_config.h`
- `project/MDK(V5)/build_log.txt`

## 修改前状态
- 直角入口已经能被边缘灰度特征预触发, 但 Phase1 必须等 `sum(ir_raw)==8` 全白才进入执行。
- 车辆动态过直角时, 灰度滤波、传感器噪声或 I2C 占用主循环都可能让“8路全白”这个窗口被错过。
- 如果预触发后一直等不到全白, 状态机会停在 `right_angle_phase==1`, 后续直角入口不会重新检测, 表现为偶发不执行直角。
- `Track_Main_Gyro()` 在一次轮询请求里会间接和直接各调用一次 `bno080_update()`, 增加 I2C 占用和漏采短暂灰度特征的概率。
- `GYRO_MAIN_POLL_PERIOD_MS` 当前为 `1U`, 高于 BNO080 旋转向量约 50Hz 报告频率的实际需求。

## 修改内容和原因
- 新增 `RIGHT_ANGLE_WHITE_MIN=7U`: Phase1 确认条件从 8 路全白改为至少 7 路白, 允许 1 路瞬时噪声黑点。
- 新增 `RIGHT_ANGLE_PRE_TIMEOUT_MS=120U` 和 `right_angle_pre_timer`: 预触发等待超时后自动回到空闲, 避免卡在等待态漏掉下一个直角。
- 进入预触发、进入执行、退出直角和初始化时都清零 `right_angle_pre_timer`。
- 串口调试输出增加 `pre:%d`, 现场可直接观察是否卡在预触发等待。
- `Track_Main_Gyro()` 和上电等待首帧处不再二次调用 `bno080_update()`, 只由 `Track_Gyro_TakePollRequest()` 消费一次。
- `GYRO_MAIN_POLL_PERIOD_MS` 从 `1U` 调整到 `10U`, 降低 I2C 对主循环灰度采样的挤占。

## 外部参考
- Pololu 论坛关于 line follower 90° 转弯的讨论建议: 检测到转弯后可先前探一小段再转, 避免过早 pivot 导致重新看到线时姿态偏斜。
- 公开论文示例中 90° 转弯使用边缘传感器组合检测左右 90° 转弯, 并进入单独转弯算法, 而不是完全依赖普通 PID。

## 测试建议
- 串口观察 `RA:is/phase t type pre`:
  - 正常直角应快速从 `RA:0/1 ... pre:n` 进入 `RA:1/2` 或 `RA:2/2`。
  - 如果 `pre` 经常涨到 120 后复位, 说明入口特征触发到了, 但全白/近全白确认仍然没到, 可继续降低车速或把 `RIGHT_ANGLE_WHITE_MIN` 临时降到 `6U` 对比。
- 连续跑 10 次同一方向直角, 记录漏触发次数和 `IR` 模式。
- 若误触发增多, 优先把 `RIGHT_ANGLE_WHITE_MIN` 改回 `8U`, 或把入口特征加连续确认。

## 编译验证
- Keil MDK 命令行构建通过: `0 Error(s), 0 Warning(s)`。
