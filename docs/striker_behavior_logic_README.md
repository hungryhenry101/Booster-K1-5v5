# Striker 行为树逻辑说明文档

## 文档概述

本文档详细说明了足球机器人Striker（前锋）的行为树架构、各个游戏状态下的行为逻辑，以及行为树节点的执行流程。

所有的行为树文件位于 src/brain/behavior_trees/

---

## 目录

1. [整体架构](#整体架构)
2. [游戏状态机](#游戏状态机)
3. [Striker行为树详解](#striker行为树详解)
   - [StrikerPlay 正常比赛行为](#strikerplay-正常比赛行为)
   - [StrikerFreekick 任意球行为](#strikerfreekick-任意球行为)
4. [核心决策机制](#核心决策机制)
5. [辅助行为树](#辅助行为树)
6. [参数配置](#参数配置)
7. [行为流程图](#行为流程图)

---

## 整体架构

### 主行为树 (game.xml)

Striker的行为作为整个机器人行为树的一部分，在主行为树中被调用：

```
MainTree
└── ReactiveSequence (control_state == 3 && !go_manual)
    └── ReactiveSequence (!gc_is_under_penalty)  # 非罚时状态
        ├── ReactiveSequence (gc_game_sub_state_type == 'NONE')  # 正常比赛子状态
        │   ├── ReactiveSequence (gc_game_state == 'INITIAL')   # 入场站位
        │   ├── ReactiveSequence (gc_game_state == 'READY')      # 走到起始位置
        │   ├── ReactiveSequence (gc_game_state == 'SET')       # 等待比赛开始
        │   ├── ReactiveSequence (gc_game_state == 'PLAY')      # 比赛进行中
        │   │   └── SubTree ID="StrikerPlay" _autoremap="true" _while="player_role == 'striker'"
        │   └── ReactiveSequence (gc_game_state == 'END')       # 比赛结束
        └── ReactiveSequence (gc_game_sub_state_type == 'FREE_KICK' && gc_game_state == 'PLAY')
            ├── ReactiveSequence (gc_game_sub_state == 'STOP')     # 任意球停止
            ├── ReactiveSequence (gc_game_sub_state == 'GET_READY')  # 任意球准备
            │   └── SubTree ID="StrikerFreekick" _autoremap="true" _while="player_role == 'striker'"
            └── ReactiveSequence (gc_game_sub_state == 'SET')      # 任意球就绪
```

### 控制模式

系统支持三种控制模式（通过手柄切换）：

| control_state | 模式 | 说明 |
|---------------|------|------|
| 1 | Manual | 人工接管，使用SimpleChase和Kick |
| 2 | Locate | 定位模式，入场并校准 |
| 3 | Auto | 自动模式（正常比赛） |

---

## 游戏状态机

### Game State 主状态

| 状态 | 说明 | Striker行为 |
|------|------|-------------|
| **INITIAL** | 初始状态，在入场位置站好 | 定位 + 找球 |
| **READY** | 准备状态，走到场内起始位置 | 移动到起始位置 + 定位 |
| **SET** | 就绪状态，等待比赛开始 | 找球 + 定位 + 停止 |
| **PLAY** | 比赛进行中 | 执行StrikerPlay行为树 |
| **END** | 比赛结束 | 停止 |

### Game Sub State 子状态

| 子状态 | 说明 | 出现条件 |
|--------|------|----------|
| **NONE** | 正常比赛 | 无特殊事件 |
| **TIMEOUT** | 比赛暂停 | 暂停时，game state必须为INITIAL |
| **FREE_KICK** | 任意球 | 任意球事件 |

### Free Kick Sub State 任意球子状态

| 子状态 | 说明 | Striker行为 |
|--------|------|-------------|
| **STOP** | 停止 | 找球 + 定位 + 停止 |
| **GET_READY** | 准备 | 移动到任意球位置 |
| **SET** | 就绪 | 找球 + 定位 + 停止 |

---

## Striker行为树详解

### StrikerPlay 正常比赛行为

**文件位置**: `subtree_striker_play.xml`

#### 行为树结构

```
StrikerPlay
└── ReactiveSequence
    ├── SelfLocate mode="trust_direction"                    # 持续定位（信任方向）
    ├── SubTree ID="Locate" _autoremap="true" _while="decision!='find'"  # 找球时跳过定位
    │
    ├── ReactiveSequence _while="wait_for_opponent_kickoff"  # 等待对方开球
    │   ├── SubTree ID="CamFindAndTrackBall" _autoremap="true"
    │   └── SetVelocity                                      # 停止移动
    │
    └── ReactiveSequence _while="!wait_for_opponent_kickoff"  # 正常比赛
        ├── Sequence _while="ball_out"                        # 球出界处理
        │   ├── SubTree ID="CamFindAndTrackBall" _autoremap="true"
        │   ├── GoBackInField                                  # 返回场内
        │   └── SubTree ID="Locate" _autoremap="true" _while="decision!='find'"
        │
        └── ReactiveSequence _while="!ball_out"                # 球在场内
            ├── SubTree ID="CamFindAndTrackBall" _autoremap="true"
            ├── CalcKickDir                                    # 计算踢球方向
            └── Sequence                                       # 决策执行序列
                ├── ReactiveSequence
                │   ├── StrikerDecide decision_out="{decision}" chase_threshold="1.0"
                │   └── SubTree ID="FindBall" _while="decision=='find'" _autoremap="true"
                │
                ├── Assist _while="decision == 'assist'" vx_limit="0.5" vy_limit="0.5"
                ├── Chase   _while="decision == 'chase'"  vx_limit="1.7" vy_limit="0.5" vtheta_limit="0.5" dist="0.1" safe_dist="0.5"
                ├── Adjust  _while="decision == 'adjust'" turn_threshold="0.5" range="0.4" vx_limit="0.7" vy_limit="0.6" vtheta_limit="1.5"
                ├── Kick    _while="decision == 'kick'"  speed_limit="0.9" min_msec_kick="1000" msecs_stablize="1000"
                └── Kick    _while="decision == 'cross'" speed_limit="0.6" min_msec_kick="600"
```

#### 行为流程详解

**阶段1: 持续定位**
- 使用`SelfLocate`模式`trust_direction`进行持续定位
- 该模式信任当前朝向，允许一定漂移，适合比赛中的轻量级定位

**阶段2: 等待对方开球 (`wait_for_opponent_kickoff=true`)**
- 持续跟踪球（`CamFindAndTrackBall`）
- 保持静止（`SetVelocity`）
- 等待对方开球后，标志位变为false，进入正常比赛逻辑

**阶段3: 球出界处理 (`ball_out=true`)**
- 持续跟踪球
- 调用`GoBackInField`返回场内
- 重新定位

**阶段4: 核心决策循环**

1. **感知与决策**
   - `CamFindAndTrackBall`: 相机找球或跟踪球
   - `CalcKickDir`: 计算最佳踢球方向（射门/传中/防守）

2. **StrikerDecide 决策节点**
   根据当前情况输出决策`decision`:

   | decision | 条件 | 行为 | 参数 |
   |----------|------|------|------|
   | **find** | 不知道球位置或不可靠 | 找球 | - |
   | **assist** | 队友控球 (`tmImLead=false`) | 辅助防守 | vx=0.5, vy=0.5 |
   | **chase** | 球距离 > 1.0m | 追球 | vx=1.7, vy=0.5, vθ=0.5, dist=0.1, safe_dist=0.5 |
   | **adjust** | 球在附近但角度/位置不合适 | 调整位置 | range=0.4, vx=0.7, vy=0.6, vθ=1.5 |
   | **kick** | 角度好且距离合适 | 踢球 | speed=0.9, time=1000ms, stabilize=1000ms |
   | **cross** | 传中 | 传中踢球 | speed=0.6, time=600ms |
   | **safe_shoot** | 低威胁且角度好 | 安全射门 | 同kick |

   **StrikerDecide决策逻辑** (详见brain_tree_README.md):
   ```
   if (!ball_location_known && !tm_ball_pos_reliable)
       decision = "find"
   else if (!tmImLead)
       decision = "assist"
   else if (ballRange > chase_threshold)
       decision = "chase"
   else if (angleGoodForKick && !avoidKick && ballDetected &&
            |ballYaw| < KICK_THETA_RANGE && ballRange < KICK_RANGE)
       if (kickType == "cross")
           decision = "cross"
       else if (threatLevel < threatThreshold)
           decision = "safe_shoot"
       else
           decision = "kick"
   else
       decision = "adjust"
   ```

3. **FindBall 找球子树** (`decision=='find'`)
   ```
   FindBall
   └── Sequence
       ├── ReactiveSequence
       │   ├── GoBackInField           # 检查是否出界
       │   └── CamFastScan             # 快速头部扫描
       ├── TurnOnSpot rad="3.14" towards_ball="true"  # 原地转向记忆中球的方向
       ├── ReactiveSequence
       │   ├── GoBackInField
       │   └── CamFastScan
       └── ReactiveSequence
           ├── GoToReadyPosition vx_limit="0.7"  # 回到准备位置
           └── Sleep msec="5000"
   ```

4. **Assist 辅助防守** (`decision=='assist'`)
   - 根据排名移动到防守位置
   - Rank 1: 球后方2m，可以挡住球的位置
   - Rank 2/3: 罚球区附近
   - 支持障碍物避免
   - 速度限制: vx=0.5, vy=0.5

5. **Chase 追球** (`decision=='chase'`)
   - 智能追球，支持circle back和chase两种模式
   - 当机器人在球后方时，circle back绕到球前方
   - 正常追球时，目标位置为球前方0.1m，沿kickDir方向
   - 使用sigmoid函数平滑过渡，距离远时优先转向
   - 安全距离: 0.5m
   - 速度限制: vx=1.7, vy=0.5, vθ=0.5

6. **Adjust 调整位置** (`decision=='adjust'`)
   - 调整到球后方range距离，沿kickDir方向
   - 角度差过大时(>0.5rad)仅横向移动
   - 角度差不大时平移+转向
   - 目标距离: 0.4m
   - 速度限制: vx=0.7, vy=0.6, vθ=1.5

7. **Kick 踢球** (`decision=='kick'`)
   - 分为stabilize和kick两个阶段
   - Stabilize: 后退稳定1000ms
   - Kick: 向球方向移动，速度0.9，持续时间1000ms + 距离相关
   - 逐渐加速，避免乌龙
   - 支持障碍物避免

8. **Cross 传中** (`decision=='cross'`)
   - 低速踢球，速度0.6
   - 持续时间600ms
   - 不需要稳定阶段

---

### StrikerFreekick 任意球行为

**文件位置**: `subtree_striker_freekick.xml`

#### 行为树结构

```
StrikerFreekick
└── ReactiveSequence
    ├── ReactiveSequence _while="gc_is_sub_state_kickoff_side"  # 我方任意球
    │   ├── CamTrackBall _while="ball_location_known"
    │   ├── CalcKickDir
    │   ├── SubTree ID="FindBall" _while="!(ball_location_known || tm_ball_pos_reliable)"
    │   └── GoToFreekickPosition _while="ball_location_known || tm_ball_pos_reliable" side="attack"
    │
    └── ReactiveSequence _while="!gc_is_sub_state_kickoff_side"  # 对方任意球
        ├── CamTrackBall _while="ball_location_known"
        ├── CalcKickDir
        ├── SubTree ID="FindBall" _while="!(ball_location_known || tm_ball_pos_reliable)"
        └── GoToFreekickPosition _while="ball_location_known || tm_ball_pos_reliable" side="defense"
```

#### 行为流程详解

**我方任意球** (`gc_is_sub_state_kickoff_side=true`):

1. **跟踪球**: 知道球位置时跟踪，不知道时找球
2. **计算踢球方向**: `CalcKickDir`
3. **移动到进攻位置**: `GoToFreekickPosition side="attack"`
   - Rank 0: 球后方attack_dist距离，沿kickDir方向
   - Rank 1: 球后方2m，沿防守方向
   - Rank 2/3: 罚球区角落
   - 速度限制: vx=0.7, vy=0.4

**对方任意球** (`gc_is_sub_state_kickoff_side=false`):

1. **跟踪球**: 知道球位置时跟踪，不知道时找球
2. **计算踢球方向**: `CalcKickDir`
3. **移动到防守位置**: `GoToFreekickPosition side="defense"`
   - Rank 0: 球后方2m，沿防守方向
   - Rank 1: 球后方1.5m，沿防守方向
   - Rank 2/3: 罚球区角落
   - 速度限制: vx=0.7, vy=0.4

---

## 核心决策机制

### StrikerDecide 决策节点

StrikerDecide是StrikerPlay的核心决策节点，决定了机器人在不同情况下应该执行的行为。

#### 决策流程图

```
开始
  │
  ├─→ 不知道球位置？ ──→ decision = "find"
  │
  ├─→ 队友控球？ ──→ decision = "assist"
  │
  ├─→ 球距离 > chase_threshold？ ──→ decision = "chase"
  │
  ├─→ 角度合适 + 非碰撞 + 看见球 + 角度范围 + 距离范围？
  │   │
  │   ├─→ kickType == "cross"？ ──→ decision = "cross"
  │   │
  │   ├─→ 威胁等级 < 阈值？ ──→ decision = "safe_shoot"
  │   │
  │   └─→ decision = "kick"
  │
  └─→ decision = "adjust"
```

#### 决策条件详解

1. **不知道球位置** (`!ball_location_known && !tm_ball_pos_reliable`)
   - 自己不知道球位置
   - 队友球位置也不可靠

2. **队友控球** (`!tmImLead`)
   - 通过双机通讯判断，`is_lead=false`表示队友控球
   - 自己转为辅助防守角色

3. **追球条件** (`ballRange > chase_threshold`)
   - chase_threshold = 1.0m
   - 球在较远位置，需要先追球

4. **踢球条件**:
   - `angleGoodForKick`: 机器人-球-门柱角度适合踢球
   - `!avoidKick`: 没有碰撞风险
   - `ballDetected`: 看见球
   - `|ballYaw| < KICK_THETA_RANGE`: 球在机器人前方（约1.5弧度范围内）
   - `ballRange < KICK_RANGE`: 球在踢球范围内（约1.0m）

5. **传中条件** (`kickType == "cross"`)
   - 由CalcKickDir决定
   - 两个门柱角度差小于cross_threshold时选择传中

6. **安全射门** (`threatLevel < threatThreshold`)
   - 威胁等级低，可以稳健射门

7. **调整位置** (default)
   - 球在附近但角度或位置不理想
   - 微调位置到合适的踢球点

#### 输出决策与行为的映射

| 决策 | 行为节点 | 目标 | 关键参数 |
|------|----------|------|----------|
| find | FindBall | 找到球 | - |
| assist | Assist | 防守位置 | vx=0.5, vy=0.5 |
| chase | Chase | 追球 | vx=1.7, vy=0.5, vθ=0.5 |
| adjust | Adjust | 调整到踢球点 | range=0.4, vx=0.7 |
| kick | Kick | 踢球 | speed=0.9, 1000ms |
| cross | Kick | 传中 | speed=0.6, 600ms |

---

## 辅助行为树

### CamFindAndTrackBall 相机找球和跟踪球

**文件位置**: `subtree_cam_find_and_track_ball.xml`

```
CamFindAndTrackBall
└── Sequence
    └── IfThenElse condition="ball_location_known || tm_ball_pos_reliable"
        ├── [Yes] → CamTrackBall          # 跟踪球
        └── [No]  → CamFindBall           # 找球
```

**行为逻辑**:
- 知道球位置（自己或队友）→ 跟踪球
- 不知道球位置 → 找球（6步扫描序列）

### Locate 定位子树

**文件位置**: `subtree_locate.xml`

```
Locate
└── Sequence
    ├── SelfLocate mode="trust_direction"
    ├── SelfLocate1M     # 单标记定位
    ├── SelfLocate2T     # 双T标记定位
    ├── SelfLocatePT     # P-T标记定位
    ├── SelfLocateLT     # L-T标记定位
    ├── SelfLocate2X     # 双X标记定位
    └── SelfLocateBorder # 边线定位
```

**定位策略**:
- 使用多种特征组合进行位置校正
- 每个定位节点都有独立的时间间隔和容差参数
- 按优先级从精确到粗略排列

### AutoGetUpAndLocate 自动起身和定位

**文件位置**: `subtree_auto_standup_and_locate.xml`

```
AutoGetUpAndLocate
└── Sequence
    └── CheckAndStandUp    # 检查跌倒并尝试起身
```

**功能**:
- 自动检测是否跌倒
- 在阻尼模式(damping)时调用standUp()
- 限制最大尝试次数
- 起身成功后重置状态

---

## 参数配置

### StrikerPlay 参数配置

| 参数 | 节点 | 值 | 说明 |
|------|------|-----|------|
| chase_threshold | StrikerDecide | 1.0 | 追球距离阈值(m) |
| vx_limit (chase) | Chase | 1.7 | 前进速度限制(m/s) |
| vy_limit (chase) | Chase | 0.5 | 横向速度限制(m/s) |
| vtheta_limit (chase) | Chase | 0.5 | 角速度限制(rad/s) |
| dist (chase) | Chase | 0.1 | 追球时目标距离(m) |
| safe_dist (chase) | Chase | 0.5 | 安全距离(m) |
| turn_threshold (adjust) | Adjust | 0.5 | 转向阈值(rad) |
| range (adjust) | Adjust | 0.4 | 目标距离(m) |
| vx_limit (adjust) | Adjust | 0.7 | 前进速度限制(m/s) |
| vy_limit (adjust) | Adjust | 0.6 | 横向速度限制(m/s) |
| vtheta_limit (adjust) | Adjust | 1.5 | 角速度限制(rad/s) |
| speed_limit (kick) | Kick | 0.9 | 踢球速度(m/s) |
| min_msec_kick (kick) | Kick | 1000 | 最小踢球时间(ms) |
| msecs_stablize (kick) | Kick | 1000 | 稳定时间(ms) |
| speed_limit (cross) | Kick | 0.6 | 传中速度(m/s) |
| min_msec_kick (cross) | Kick | 600 | 传中时间(ms) |
| vx_limit (assist) | Assist | 0.5 | 前进速度限制(m/s) |
| vy_limit (assist) | Assist | 0.5 | 横向速度限制(m/s) |

### StrikerFreekick 参数配置

| 参数 | 节点 | 值 | 说明 |
|------|------|-----|------|
| vx_limit | GoToFreekickPosition | 0.7 | 前进速度限制(m/s) |
| vy_limit | GoToFreekickPosition | 0.4 | 横向速度限制(m/s) |

---

## 行为流程图

### 完整的Striker行为流程

```
┌─────────────────────────────────────────────────────────────┐
│                    MainTree 启动                            │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ├─→ control_state == 1 (Manual)
                          │     └─→ SimpleChase + Kick
                          │
                          ├─→ control_state == 2 (Locate)
                          │     └─→ CamScanField + SelfLocateEnterField
                          │
                          └─→ control_state == 3 (Auto) ← 正常模式
                                │
                                ├─→ CheckAndStandUp
                                │
                                ├─→ gc_is_under_penalty?
                                │     ├─→ Yes: 罚时状态
                                │     └─→ No: 非罚时状态
                                │           │
                                │           ├─→ gc_game_sub_state == 'TIMEOUT'
                                │           │     └─→ SetVelocity + CamFindAndTrackBall
                                │           │
                                │           ├─→ gc_game_sub_state == 'FREE_KICK'
                                │           │     ├─→ STOP: SetVelocity
                                │           │     ├─→ GET_READY: StrikerFreekick
                                │           │     └─→ SET: SetVelocity
                                │           │
                                │           └─→ gc_game_sub_state == 'NONE'
                                │                 │
                                │                 ├─→ gc_game_state == 'INITIAL'
                                │                 │     └─→ Locate + CamFindAndTrackBall
                                │                 │
                                │                 ├─→ gc_game_state == 'READY'
                                │                 │     └─→ GoToReadyPosition + Locate
                                │                 │
                                │                 ├─→ gc_game_state == 'SET'
                                │                 │     └─→ CamFindAndTrackBall + SetVelocity + Locate
                                │                 │
                                │                 ├─→ gc_game_state == 'PLAY' ← 比赛进行中
                                │                 │     └─→ StrikerPlay (player_role == 'striker')
                                │                 │           │
                                │                 │           ├─→ SelfLocate (trust_direction)
                                │                 │           ├─→ Locate (while decision != 'find')
                                │                 │           │
                                │                 │           ├─→ wait_for_opponent_kickoff?
                                │                 │           │     ├─→ Yes: CamFindAndTrackBall + SetVelocity
                                │                 │           │     └─→ No: 正常比赛
                                │                 │           │           │
                                │                 │           │           ├─→ ball_out?
                                │                 │           │           │     ├─→ Yes: CamFindAndTrackBall + GoBackInField + Locate
                                │                 │           │           │     └─→ No: 决策循环
                                │                 │           │           │           │
                                │                 │           │           │           ├─→ CamFindAndTrackBall
                                │                 │           │           │           ├─→ CalcKickDir
                                │                 │           │           │           │
                                │                 │           │           │           └─→ StrikerDecide
                                │                 │           │           │                 │
                                │                 │           │           │                 ├─→ find → FindBall
                                │                 │           │           │                 │       ├─→ GoBackInField + CamFastScan
                                │                 │           │           │                 │       ├─→ TurnOnSpot + CamFastScan
                                │                 │           │           │                 │       └─→ GoToReadyPosition
                                │                 │           │           │                 │
                                │                 │           │           │                 ├─→ assist → Assist
                                │                 │           │           │                 │       └─→ 防守位置 (vx=0.5, vy=0.5)
                                │                 │           │           │                 │
                                │                 │           │           │                 ├─→ chase → Chase
                                │                 │           │           │                 │       └─→ 追球 (vx=1.7, vy=0.5, vθ=0.5)
                                │                 │           │           │                 │
                                │                 │           │           │                 ├─→ adjust → Adjust
                                │                 │           │           │                 │       └─→ 调整位置 (range=0.4, vx=0.7)
                                │                 │           │           │                 │
                                │                 │           │           │                 ├─→ kick → Kick
                                │                 │           │           │                 │       ├─→ Stabilize (1000ms)
                                │                 │           │           │                 │       └─→ Kick (speed=0.9, 1000ms+)
                                │                 │           │           │                 │
                                │                 │           │           │                 └─→ cross → Kick
                                │                 │           │           │                       └─→ 传中 (speed=0.6, 600ms)
                                │                 │           │           │
                                │                 │           │           └─→ (循环)
                                │                 │           │
                                │                 │           └─→ (循环)
                                │                 │
                                │                 └─→ gc_game_state == 'END'
                                │                       └─→ SetVelocity
                                │
                                └─→ (循环)
```

---

## 关键特性总结

### 1. 层次化决策

Striker行为采用三层决策架构：
- **顶层**: 游戏状态（INITIAL/READY/SET/PLAY/END）
- **中层**: 子状态（NONE/TIMEOUT/FREE_KICK）和特殊条件
- **底层**: StrikerDecide战术决策

### 2. 实时反应

使用`ReactiveSequence`实现实时响应：
- 每次tick重新评估所有条件
- 状态变化立即切换行为
- 适应动态比赛环境

### 3. 容错机制

- **找球失败**: 多阶段找球策略（扫描→转向→扫描→回准备位置）
- **定位失败**: 多种定位方法组合（标记/边线）
- **跌倒恢复**: 自动检测并尝试起身

### 4. 协作能力

- **双机通讯**: 通过`is_lead`标志判断控球权
- **角色切换**: 根据`player_role`执行不同行为
- **辅助防守**: 队友控球时转为辅助角色

### 5. 策略多样性

- **进攻模式**: chase → adjust → kick/cross
- **防守模式**: assist（移动到防守位置）
- **找球模式**: 多阶段找球策略
- **任意球**: 根据攻守方移动到不同位置

---

## 行为树设计原则

### 1. 单一职责

每个节点只负责一个明确的功能：
- `StrikerDecide`: 只做决策
- `Chase`: 只负责追球
- `Kick`: 只负责踢球

### 2. 可复用性

子树在不同上下文中复用：
- `CamFindAndTrackBall`: 多处使用
- `Locate`: 多处使用
- `FindBall`: 找球逻辑独立

### 3. 参数化

行为参数在XML中配置，易于调整：
- 速度限制
- 距离阈值
- 时间参数

### 4. 状态驱动的切换

通过blackboard变量控制行为切换：
- `decision`: 决策类型
- `ball_location_known`: 球位置状态
- `wait_for_opponent_kickoff`: 等待开球标志
- `ball_out`: 球出界标志

---

## 调试和优化建议

### 1. 日志监控

关键节点的日志输出：
- `StrikerDecide`: 输出决策、球距离、角度等
- `Locate`: 输出定位结果、残差
- `Kick`: 输出踢球状态

### 2. 参数调优

根据实际表现调整关键参数：
- `chase_threshold`: 控制追球触发时机
- `vx_limit/vy_limit/vtheta_limit`: 调整运动速度
- `speed_limit` (kick): 调整踢球力度

### 3. 决策逻辑优化

根据比赛需求调整`StrikerDecide`逻辑：
- 调整威胁等级阈值
- 增加或减少决策条件
- 修改优先级顺序

### 4. 定位策略

优化定位节点顺序和参数：
- 调整`msecs_interval`: 控制定位频率
- 调整`max_dist/max_drift`: 控制定位精度

---

## 总结

Striker行为树实现了一个完整的足球前锋智能决策系统，具有以下特点：

1. **完整的状态管理**: 覆盖所有游戏状态和子状态
2. **智能的决策机制**: 基于多种条件做出战术决策
3. **灵活的行为切换**: 实时响应环境变化
4. **健壮的容错能力**: 多层次容错机制
5. **高效的协作能力**: 支持双机协作
6. **可配置的参数**: 易于调整和优化

通过理解这个行为树架构，可以更好地理解机器人足球的智能决策逻辑，为后续的优化和扩展提供基础。
