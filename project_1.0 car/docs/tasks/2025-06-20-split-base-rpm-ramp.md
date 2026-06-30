# 拆分 BASE_RPM_RAMP 为两个独立斜坡参数

**日期**: 2025-06-21

## 涉及文件
- `bsp/track_config.h` — 宏定义拆分
- `bsp/track.c` — 斜坡逻辑 + 出弯基础转速策略修改

## 修改前状态
`BASE_RPM_RAMP` (0.1f) 同时服务于两个不同的场景:
1. **上电起步** — `base_rpm_current` 从 0 → BASE_RPM (80)
2. **退出直角恢复** — `base_rpm_current` 从 CORNER_TURN_RPM → BASE_RPM

两段复用同一段代码 (track.c:487-493)，无法用不同的速率。

## 修改内容

### track_config.h
```
# 删除:
#define BASE_RPM_RAMP       0.1f   //上电退出直角加速

# 新增:
#define RAMP_STARTUP        0.1f   //上电起步斜坡 (0→BASE_RPM)
#define RAMP_POST_CORNER    0.0001f  //退出直角恢复斜坡 (corner_min 20→BASE_RPM)
```

### track.c — 两处关键改动

1. **斜坡速率选择** — `base_rpm_current` 按方向区分:
   - `< BASE_RPM` → `RAMP_STARTUP` (上电起步)
   - `> BASE_RPM` → `RAMP_POST_CORNER` (出弯恢复)
   - `corner_min_current` 上升: `base_rpm_current` 已到 BASE_RPM → 出弯恢复, 用 `RAMP_POST_CORNER`; 否则上电起步, 用 `RAMP_STARTUP`

2. **出弯基础转速** — 新增 `effective_base` 变量:
   - 当 `base_rpm_current >= BASE_RPM && corner_min_current < BASE_RPM` (出弯恢复态)
   - `effective_base = corner_min_current` (用内轮钳位值做基础转速)
   - 使轮速跟随 `corner_min_current` 从 20→80 极缓慢爬升
   - 其他状态: `effective_base = base_rpm_current` (行为不变)

## 修改原因
注释已指出问题: "这个参数分开两个状态机用不同的参数"。原共用 0.1f 在退出直角时恢复过猛。改用 `corner_min_current` 做有效基础转速后，RAMP_POST_CORNER 直接控制轮速恢复速率。

## 测试建议
1. **上电起步**: 观察车轮是否从 0 RPM 平滑加速到 80 RPM（与改动前行为一致）
2. **直角转弯退出**: 出弯后轮速从约 20 RPM 开始, 以 RAMP_POST_CORNER 速率缓慢爬升到 80 RPM
3. 恢复过慢 → 调大 `RAMP_POST_CORNER`; 起步过慢 → 调大 `RAMP_STARTUP`
