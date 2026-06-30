# 2026-06-20 - 修复直角转弯后循迹摇头

## 日期
- 2026-06-20

## 涉及文件
- `bsp/track.c` — err_filtered 提升为文件作用域 + Phase 3 全状态清零

## 修改前状态
- 上电直道循迹正常，执行一次直角转弯后开始摇头振荡
- Phase 3 退出时只清零 `PID_SumErr`，其他状态未清理

## 根因分析

直角转弯执行期间（Phase 1），`Track_PID_Calc` 被跳过，但以下状态变量被转弯过程污染：

| 变量 | 污染原因 | 退出后影响 |
|------|----------|-----------|
| `err_filtered` (原static) | 保持转弯前旧值 | EMA从旧值缓慢过渡，前几十ms纠偏方向错误 |
| `pid_filtered_output` | 保持转弯前旧值 | PID输出从旧值起步，延迟响应 |
| `Left_Integral` / `Right_Integral` | 转弯差速时积分饱和到±1000 | 退出后积分未释放，PWM过冲→超调→振荡 |
| `PID_LastErr` | 保持转弯前旧值 | D项首帧跳变 |
| `Left_FilteredEnc` / `Right_FilteredEnc` | 保持转弯前编码器滤波值 | 速度环EMA从错误状态起步 |
| `Left_LastBias` / `Right_LastBias` | 保持转弯前偏差 | 速度环D项跳变 |

## 修改内容

### 1. err_filtered 提升为文件作用域
- 从 `Track_Calc_Err()` 内的 `static float` 移到文件作用域
- 使 Phase 3 可以清零该状态

### 2. Phase 3 全状态清零
```c
// 位置PID状态
PID_SumErr          = 0.0f;
PID_LastErr         = 0.0f;
PID_Err             = 0.0f;
pid_filtered_output = 0.0f;
err_filtered        = 0.0f;

// 速度环状态
Left_Integral   = 0.0f;
Right_Integral  = 0.0f;
Left_LastBias   = 0.0f;
Right_LastBias  = 0.0f;
Left_FilteredEnc  = 0.0f;
Right_FilteredEnc = 0.0f;
```

## 测试建议
- 直道 → 直角 → 直道，观察转弯后是否还会摇头
- 如果转弯后仍有轻微抖动但很快收敛，说明方向对，微调PID参数即可
- 如果转弯后车轮短暂不同步（积分从0开始重建），属于正常冷启动效应
