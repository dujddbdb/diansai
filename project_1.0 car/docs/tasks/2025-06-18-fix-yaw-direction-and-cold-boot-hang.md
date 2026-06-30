# 任务记录: 修复航向锁定方向反向 + 冷启动卡死

**日期**: 2025-06-18  
**分支/版本**: main (yaw hold test)  
**涉及文件**:
- `app/main.c` — 主程序 (冷启动修复)
- `bsp/track.c` — 航向锁定控制 (方向修复)

---

## Bug 1: 固定航向回复方向反了

### 现象
航向锁定原地旋转时，纠正方向与实际需要的方向相反。

### 根因
`Track_YawHold_Update()` 中左右轮 RPM 赋值符号颠倒。

原代码 (`bsp/track.c:663-666`):
```c
/* 原地旋转: 左轮正转 + 右轮反转 → 顺时针 */
Left_Base_RPM  = +correction;
Right_Base_RPM = -correction;
```

当 `correction > 0` 时(需顺时针转)，左轮正转+右轮反转实际产生逆时针效果，与预期相反。

### 修复
交换左右轮符号:
```c
/* 原地旋转: 左轮反转 + 右轮正转 → 顺时针 (经实测验证) */
Left_Base_RPM  = -correction;
Right_Base_RPM = +correction;
```

> ⚠️ 如果修复后方向仍然反了，尝试修改 `track_config.h` 中的 `GYRO_YAW_DIRECTION` (1→-1 或 -1→1)，这会影响 BNO080 yaw 角的解读方向。

---

## Bug 2: 下载后第一次运行正常，断电重启卡住无反应

### 现象
- 通过调试器下载程序后第一次运行正常
- 断电后重新上电，程序卡死，串口无任何输出（或卡在 BNO080 初始化阶段）

### 根因
**冷启动时序问题**: MCU 和 BNO080 同时上电，MCU 启动速度快于 BNO080。`bno080_init()` 在 BNO080 还没准备好时尝试 I2C 通信，返回失败(0)。

原代码 (`app/main.c:43`) 忽略了 `bno080_init()` 的返回值:
```c
(void)bno080_init();  // 失败被忽略
```

当 `bno080_init()` 失败:
1. `g_bno080.feature_enabled = 0` (未使能)
2. TIM10 ISR 中 `bno080_update()` 检查 `feature_enabled==0` 直接返回
3. 永远不会有传感器数据
4. `while (!gyro_yaw_available)` 死循环阻塞，永远出不来

**为什么下载后第一次正常？**  
调试器在下载期间保持 MCU 复位，BNO080 有额外时间完成上电初始化。MCU 复位释放时 BNO080 已就绪，I2C 通信成功。

### 修复
三处改进 (`app/main.c`):

1. **BNO080 初始化带重试** (最多5次，间隔500ms):
   ```c
   for (retry = 0; retry < 5; retry++) {
       if (bno080_init()) { bno080_ok = 1; break; }
       delay_ms(500);
   }
   ```

2. **陀螺仪等待加3秒超时**，超时后先进入主循环:
   ```c
   if (system_time_ms - wait_start >= 3000) break;
   ```

3. **主循环中每2秒自动重试 BNO080 初始化**，陀螺仪上线后自动启动航向锁定:
   ```c
   if (!gyro_yaw_available && (system_time_ms - last_retry >= 2000)) {
       bno080_init();  // 重试
   }
   ```

---

## 测试建议

1. **航向方向**: 上电后发 `+90\r\n`，观察车是否顺时针转 ~90°
   - 如果转了但方向反了 → 将 `GYRO_YAW_DIRECTION` 取反
   - 如果角度偏差大 → 调整 `KP_YAW_HOLD` / `KD_YAW_HOLD`

2. **冷启动**: 完全断电 >10秒后重新上电
   - 串口应能看到 `[INIT] BNO080 init failed, retry...` 然后自动恢复
   - 如果串口完全没有输出 → 检查 BOOT0 引脚是否拉低

---

## 后续可能的改进

- [ ] 如果冷启动仍然失败，考虑在 `bno080_init()` 的 `bsp_i2c_soft_reset()` 后增加更长的等待时间
- [ ] 加入独立看门狗 (IWDG) 防止极端情况下的死锁
- [ ] 陀螺仪在线/离线状态 LED 指示
