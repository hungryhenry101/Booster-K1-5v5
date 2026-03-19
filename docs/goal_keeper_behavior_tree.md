# GoalKeeper 行为树逻辑说明文档

## 1. 概述

GoalKeeper（守门员）机器人使用行为树（Behavior Tree）进行决策控制，主要包含两个行为树：
- **GoalKeeperPlay**: 正常比赛模式
- **GoalKeeperFreekick**: 任意球模式

## 2. 行为树文件位置

- 正常比赛: `src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml`
- 任意球: `src/brain/behavior_trees/subtrees/subtree_goal_keeper_freekick.xml`
- 实现代码: `src/brain/src/brain_tree.cpp`

## 3. GoalKeeperPlay - 正常比赛行为树

### 3.1 整体结构

```
GoalKeeperPlay
├── SelfLocate (自定位)
├── Locate (定位子树)
├── 等待对方开球
│   ├── CamFindAndTrackBall (寻找并追踪球)
│   └── SetVelocity (设置速度)
└── 已方开球或对方已经开球
    ├── 球出界处理
    │   ├── CamFindAndTrackBall
    │   ├── GoBackInField (回场内)
    │   └── Locate
    └── 正常比赛
        ├── Attack模式
        │   ├── CamFindAndTrackBall
        │   ├── GoalieDecide (决策节点)
        │   ├── GoToGoalBlockingPosition (寻找位置)
        │   ├── Chase (追球)
        │   ├── Adjust (调整位置)
        │   └── Kick (踢球)
        └── Guard模式
            ├── CamFindAndTrackBall
            ├── GoToReadyPosition (准备位置)
            └── GoToGoalBlockingPosition (阻挡位置)
```

### 3.2 核心决策节点：GoalieDecide

**文件位置**: `src/brain/src/brain_tree.cpp:1020`

**功能**: 根据球的位置、机器人姿态等因素决定守门员的下一个动作

**决策条件** (按优先级):

1. **find (寻找球)**
   - 条件: 既不知道球的位置，也不信任队友的球位置信息
   - 颜色标识: 黄色

2. **retreat (撤退/后撤)**
   - 条件: 球在球门前方（球.x > 0）
   - 颜色标识: 青色

3. **chase (追球)**
   - 条件: 球距离大于阈值（默认1.0米）
   - 特殊: 如果上一次决策是chase，阈值降为0.9米以保持连续性
   - 颜色标识: 绿色

4. **kick (踢球)**
   - 条件: 机器人到球的朝向正确（dir_rb_f在-90°到90°之间）
   - 颜色标识: 红色

5. **adjust (调整)**
   - 条件: 默认情况，需要调整朝向
   - 颜色标识: 白色

**关键参数**:
- `chase_threshold`: 追球距离阈值（默认1.0米）
- `decision_in`: 上一次的决策
- `decision_out`: 输出的新决策

### 3.3 GoToGoalBlockingPosition - 移动到阻挡位置

**文件位置**: `src/brain/src/brain_tree.cpp:629`

**功能**: 计算并移动到最佳的阻挡球射门的防守位置

**目标位置计算**:

对于GoalKeeper角色:
- X坐标: 固定在球门线前 `dist_to_goalline` 处
  ```cpp
  targetPose.x = -fieldLength/2 + dist_to_goalline
  ```

- Y坐标: 根据球的位置计算
  - 如果球在球门附近（`ball.x + fieldLength/2 < distToGoalline`）:
    - 球在右半场: 目标Y = 球门宽度的1/4
    - 球在左半场: 目标Y = -球门宽度的1/4
  - 否则: 根据相似三角形计算
    ```cpp
    targetPose.y = ball.y * distToGoalline / (ball.x + fieldLength/2)
    ```
    Y坐标被限制在禁区宽度的一半范围内

**运动控制**:
- vx, vy: 移动到目标位置
- vtheta: 转向球的朝向（乘以2.0倍以加快转身）
- 速度限制可通过参数配置

**结束条件**:
- 距离目标位置 < `dist_tolerance`
- 球的朝向角 < `theta_tolerance`

### 3.4 GoToReadyPosition - 移动到准备位置

**文件位置**: `src/brain/src/brain_tree.cpp:3011`

**功能**: 当球位置未知时，移动到默认的守门准备位置

**GoalKeeper的准备位置**:
- X: 球门线前 `goalAreaLength` 处
- Y: 0（球门中心）
- Theta: 0（面向前方）

```cpp
tx = -fieldLength/2 + goalAreaLength;
ty = 0;
ttheta = 0;
```

### 3.5 Chase - 追球节点

**功能**: 主动追击靠近球门的球

**参数配置** (在行为树中):
- `vx_limit`: 1.5 m/s (最大前进速度)
- `vy_limit`: 0.3 m/s (最大横向速度)
- `vtheta_limit`: 1.0 rad/s (最大角速度)
- `dist`: 0.4 m (停止距离)
- `safe_dist`: 0.6 m (安全距离)

### 3.6 Adjust - 调整节点

**功能**: 在近距离内调整位置，准备踢球

**参数配置**:
- `turn_threshold`: 0.2 rad (转身阈值)
- `range`: 0.3 m (调整范围)
- `vx_limit`: 0.3 m/s
- `vy_limit`: 0.2 m/s
- `vtheta_limit`: 1.0 rad/s

### 3.7 Kick - 踢球节点

**参数配置**:
- `speed_limit`: 1.2 m/s (踢球速度)
- `min_msec_kick`: 600 ms (最小踢球时长)

### 3.8 模式切换

**Guard模式 (防守)**:
- 当球位置未知时，移动到准备位置
- 当球位置已知时，移动到阻挡位置
- 位置响应式更新

**Attack模式 (进攻/主动防守)**:
- 持续追踪球
- 通过GoalieDecide决策具体行为
- 可以主动追球和踢球

## 4. GoalKeeperFreekick - 任意球行为树

### 4.1 整体结构

```
GoalKeeperFreekick
├── CamFindAndTrackBall (寻找并追踪球)
├── GoToReadyPosition (球位置未知时)
├── GoToGoalBlockingPosition (球位置已知时)
└── SelfLocate (自定位)
```

### 4.2 行为逻辑

这是一个简化的行为树，主要用于任意球（对方或己方任意球）场景：

1. **持续追踪球**: 使用摄像头寻找并追踪球
2. **位置选择**:
   - 球位置未知 → 移动到准备位置
   - 球位置已知 → 移动到阻挡位置
3. **自定位**: 里程标定时进行自定位（trust_direction模式）

与正常比赛的区别：
- 没有Attack/Guard模式切换
- 不包含追球、踢球等主动行为
- 专注于站位和防守

## 5. 关键变量和参数

### 5.1 行为树入口变量

| 变量名 | 类型 | 说明 |
|--------|------|------|
| `player_role` | string | 玩家角色，值为"goal_keeper" |
| `ball_location_known` | bool | 球位置是否已知 |
| `tm_ball_pos_reliable` | bool | 队友球位置是否可靠 |
| `decision` | string | 当前决策状态 |
| `ball_out` | bool | 球是否出界 |
| `wait_for_opponent_kickoff` | bool | 是否等待对方开球 |
| `goalie_mode` | string | 守门员模式："attack" 或 "guard" |
| `odom_calibrated` | bool | 里程计是否标定 |

### 5.2 重要配置参数

这些参数通常在配置文件中定义：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `strategy.enable_auto_visual_defend` | - | 是否启用自动视觉防守 |
| `chase_threshold` | 1.0 | 追球距离阈值(米) |
| `dist_to_goalline` | - | 距离球门线的距离 |
| `dist_tolerance` | - | 位置容差 |
| `theta_tolerance` | 0.2 | 角度容差(弧度) |

## 6. 状态流转图

```
                    ┌──────────┐
                    │  初始化  │
                    └────┬─────┘
                         │
                    ┌────▼─────┐
                    │ SelfLocate│
                    └────┬─────┘
                         │
                    ┌────▼─────┐
                    │  Locate  │
                    └────┬─────┘
                         │
               ┌─────────▼─────────┐
               │ wait_for_opponent │
               │    _kickoff?      │
               └────┬──────────┬───┘
         Yes   │              │   No
         ┌──────▼──────┐      └──────┬──────────────┐
         │等待对方开球  │             │ 已方开球/对方开球 │
         │ CamTrackBall│             └──────┬───────┘
         │ SetVelocity │                    │
         └─────────────┘              ┌─────▼────┐
                               No    │ ball_out?│
                      ┌──────────────┤ (球出界) │
                      │              └────┬─────┘
                      │            Yes │    │ No
                      │         ┌──────▼─┐  │
                      │         │GoBackIn│  │
                      │         │ Field  │  │
                      │         └───┬────┘  │
                      │             │       │
                      │             └───┬───┘
                      │                 │
                      │         ┌───────▼──────┐
                      │         │ goalie_mode? │
                      │         └───────┬──────┘
                      │                 │
           ┌──────────┴───┐   ┌────────┴────────┐
           │   Guard模式   │   │   Attack模式    │
           ├───────────────┤   ├─────────────────┤
           │ CamTrackBall  │   │ CamTrackBall    │
           │ 球位置未知?   │   │ GoalieDecide    │
           │               │   └─────┬───────────┘
           │    Yes No     │         │
           │   ┌──┴──┐     │    ┌────▼──────────┐
           │   │Ready│─────┼──── │ decision?     │
           │   │Pos  │     │    └─┬─┬─┬─┬─┬──┬─┬┘
           │   └─────┘     │      │ │ │ │ │  │ │
           │               │   find│ │ │ │  │ │
           │   ┌───────┐   │  ┌───▼ │ │ │  │ │
           │   │Block  │   │  │Goal │ │ │  │ │
           │   │Pos    │   │  │Block│ │ │  │ │
           │   │Pos    │   │  │Pos  │ │ │  │ │
           │   └───────┘   │  └─────┘ │ │  │ │
           │               │      retreat│ │  │ │
           │               │   ┌───────▼ │ │  │ │
           │               │   │BlockPos│ │  │ │
           │               │   └─────────┘ │  │ │
           │               │           chase│  │ │
           │               │         ┌─────▼ │ │
           │               │         │ Chase│ │
           │               │         └──────┘ │
           │               │               adjust│
           │               │            ┌───────▼┐
           │               │            │ Adjust│
           │               │            └───────┘
           │               │                 kick
           │               │              ┌──────▼┐
           │               │              │ Kick  │
           │               │              └───────┘
```

## 7. 关键算法详解

### 7.1 阻挡位置计算算法

使用相似三角形原理计算最佳防守位置：

```
          球门线
  ─────────────────────────
    ↑                   ↑
   left               right
    ↑                   ↑
    └─────────┬─────────┘
              │
              │  targetPose.y
              │
    ┌─────────▼─────────┐
    │        Ball      │
    │   (ball.x,ball.y)│
    └──────────────────┘

计算公式:
targetPose.y = ball.y * distToGoalline / (ball.x + fieldLength/2)
```

这个算法确保守门员在球和球门之间，能够最大程度阻挡射门角度。

### 7.2 GoalieDecide决策算法

```cpp
// 伪代码
if (!knowBallPos && !teammateBallReliable) {
    decision = "find";  // 不知道球在哪
} else if (ball.x > 0) {
    decision = "retreat";  // 球在前场，后撤
} else if (ballRange > chaseThreshold) {
    decision = "chase";  // 球在追击范围内
} else if (angleToBall is good) {
    decision = "kick";  // 可以踢球
} else {
    decision = "adjust";  // 调整位置
}
```

## 8. 与其他角色的交互

### 8.1 角色切换

在 `src/brain/src/brain_tree.cpp:1162`:
```cpp
curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper")
                     : tree->setEntry<string>("player_role", "striker");
```

守门员和前锋可以动态切换角色。

### 8.2 队友信息

- 队友球位置 (`tm_ball_pos_reliable`): 用于决策时参考
- 队友成本排名 (`tmMyCostRank`): 在Assist节点中使用

## 9. 常见使用场景

### 9.1 正常比赛

1. 启动后进行自定位
2. 切换到GoalKeeperPlay行为树
3. 根据球的位置选择Guard或Attack模式
4. 持续追踪球并执行相应防守动作

### 9.2 任意球

1. 切换到GoalKeeperFreekick行为树
2. 移动到防守站位
3. 保持阻挡姿态
4. 等待任意球执行

### 9.3 球出界

1. 进入GoBackInField节点
2. 返回场内
3. 重新定位
4. 恢复正常防守

## 10. 注意事项

1. **定位准确性**: GoalKeeper的防守位置计算严重依赖准确的定位，定位错误会导致站位偏差

2. **模式切换**: Attack模式下守门员会更主动，但也可能更冒险；Guard模式更保守但可能被动

3. **参数调优**: chase_threshold、速度限制等参数需要根据场地和机器人性能调优

4. **传感器依赖**: 持续需要摄像头追踪球，球丢失会影响决策

5. **队友协作**: 在多机器人协作中，需要考虑与其他防守机器人（如Assist角色）的配合

## 11. 扩展和优化建议

1. **增加预测**: 可以加入球的轨迹预测，提前预判射门方向
2. **动态阈值**: 根据比赛情况动态调整追球和踢球的阈值
3. **风险评估**: 加入风险评估机制，在危险时刻优先封堵球门
4. **多守门员**: 支持多个守门员的协同防守策略
5. **适应性学习**: 根据对手特点调整防守策略

## 12. 相关文件清单

- 行为树定义:
  - `src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml`
  - `src/brain/behavior_trees/subtrees/subtree_goal_keeper_freekick.xml`

- 实现代码:
  - `src/brain/src/brain_tree.cpp` (GoalieDecide, GoToGoalBlockingPosition, GoToReadyPosition等节点)
  - `src/brain/include/brain_tree.h` (节点类定义)

- 配置:
  - `src/brain/src/brain_config.cpp` (配置参数)
  - `src/brain/include/brain_config.h`

- 主行为树调用:
  - `src/brain/behavior_trees/game.xml`
  - `src/brain/behavior_trees/demo.xml`

---

**文档版本**: 1.0  
**最后更新**: 2025-02-02  
**维护者**: Booster K1 5v5 Demo Team
