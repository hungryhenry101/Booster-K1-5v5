# Brain 模块完整代码解读文档

## 目录

1. [概述](#1-概述)
2. [目录结构](#2-目录结构)
3. [系统架构](#3-系统架构)
4. [核心类详解](#4-核心类详解)
5. [数据流与回调函数](#5-数据流与回调函数)
6. [行为树系统](#6-行为树系统)
7. [配置文件详解](#7-配置文件详解)
8. [通信系统](#8-通信系统)
9. [定位系统](#9-定位系统)
10. [机器人控制](#10-机器人控制)

---

## 1. 概述


`brain` 是机器人足球系统的**核心决策模块**，基于 ROS2 框架开发，负责：

- **感知数据处理**: 接收视觉识别结果、里程计、IMU 等传感器数据
- **状态估计**: 计算机器人在球场中的位置、足球位置预测
- **战术决策**: 通过行为树实现踢球、追球、防守等行为的决策
- **运动控制**: 生成速度指令控制机器人运动
- **多机协作**: 通过 UDP 通信实现队友间的信息共享和战术配合
- **裁判通信**: 与裁判机通信获取比赛状态

---

## 2. 目录结构

```
src/brain/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS2 包描述文件
├── config/                 # 配置文件目录
│   ├── config.yaml         # 主配置文件 (所有运行时参数)
│   └── config.yaml.example # 配置示例
├── launch/                 # 启动文件目录
│   └── launch.py           # ROS2 launch 文件，负责节点启动和参数加载
├── behavior_trees/         # 行为树定义
│   ├── game.xml            # 主行为树 (比赛流程控制)
│   └── subtrees/           # 子行为树
│       ├── subtree_striker_play.xml        # 前锋比赛逻辑
│       ├── subtree_goal_keeper_play.xml    # 守门员比赛逻辑
│       ├── subtree_striker_freekick.xml    # 前锋任意球逻辑
│       ├── subtree_goal_keeper_freekick.xml# 守门员任意球逻辑
│       ├── subtree_locate.xml              # 定位逻辑
│       ├── subtree_find_ball.xml           # 找球逻辑
│       ├── subtree_cam_find_and_track_ball.xml  # 摄像头找球追踪
│       └── subtree_auto_standup_and_locate.xml  # 自动起身定位
├── include/                # 头文件目录
│   ├── brain.h             # Brain 类定义 (核心决策类)
│   ├── brain_config.h      # BrainConfig 类定义 (配置管理)
│   ├── brain_data.h        # BrainData 类定义 (运行时数据)
│   ├── brain_tree.h        # BrainTree 类定义 (行为树封装)
│   ├── brain_communication.h # BrainCommunication 类定义 (通信)
│   ├── brain_log.h         # BrainLog 类定义 (日志)
│   ├── robot_client.h      # RobotClient 类定义 (机器人控制)
│   ├── locator.h           # Locator 类定义 (定位)
│   ├── types.h             # 类型定义 (数据结构、枚举)
│   ├── buffer.h            # 缓冲区工具类
│   ├── utils/              # 工具函数
│   │   ├── math.h          # 数学函数
│   │   ├── print.h         # 打印函数
│   │   └── misc.h          # 杂项函数
│   ├── RoboCupGameControlData.h  # 裁判数据结构
│   ├── SPLCoachMessage.h         # 教练消息
│   └── team_communication_msg.h  # 队内通信消息结构
├── msg/                    # 自定义消息定义
│   └── Kick.msg            # 踢球消息
└── src/                    # 源代码目录
    ├── main.cpp            # 程序入口
    ├── brain.cpp           # Brain 类实现
    ├── brain_config.cpp    # BrainConfig 类实现
    ├── brain_data.cpp      # BrainData 类实现
    ├── brain_tree.cpp      # BrainTree 类实现 (行为树节点实现)
    ├── brain_communication.cpp # 通信实现
    ├── brain_log.cpp       # 日志实现
    ├── robot_client.cpp    # 机器人控制实现
    └── locator.cpp         # 定位算法实现
```

---

## 3. 系统架构

### 3.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.cpp                                 │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  ROS2 Executor (主线程) - 处理传感器回调                  │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  Tick Loop (独立线程) - 100Hz 执行决策循环               │    │
│  │     Brain.tick() → BrainTree.tick() → BehaviorTree      │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  GameController Handler (独立线程) - 裁判消息处理        │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                          Brain 类                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │BrainConfig  │  │  BrainData  │  │  BrainLog   │             │
│  │ (静态配置)  │  │ (运行时数据)│  │ (日志系统)  │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │BrainTree    │  │RobotClient  │  │  Locator    │             │
│  │(行为树)     │  │(机器人控制) │  │ (定位系统)  │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│  ┌─────────────────────────────────────────────────┐           │
│  │           BrainCommunication                     │           │
│  │  ┌──────────────┐  ┌──────────────┐            │           │
│  │  │ 裁判通信      │  │ 队内通信      │            │           │
│  │  │ (UDP 单播)    │  │ (UDP 广播)    │            │           │
│  │  └──────────────┘  └──────────────┘            │           │
│  └─────────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│   传感器输入     │ │   执行器输出     │ │   外部通信       │
│ - 视觉检测       │ │ - 速度指令       │ │ - 裁判机         │
│ - 里程计         │ │ - 头部控制       │ │ - 队友机器人     │
│ - IMU           │ │ - 踢球动作       │ │                 │
│ - 深度相机       │ │ - 起身动作       │ │                 │
└─────────────────┘ └─────────────────┘ └─────────────────┘
```

### 3.2 数据流图

```
传感器数据流:
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│ 视觉节点      │    │ 底层驱动      │    │ 裁判接口      │
│ /detection   │    │ /odometer    │    │ /game_control│
└──────┬───────┘    └──────┬───────┘    └──────┬───────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌──────────────────────────────────────────────────────────────┐
│                    detectionsCallback                         │
│                    odometerCallback                           │
│                    gameControlCallback                        │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│                      BrainData                                │
│  - ball (足球对象)                                            │
│  - robots (机器人列表)                                        │
│  - goalposts (球门柱列表)                                     │
│  - markings (场地标记列表)                                    │
│  - fieldLines (场地线列表)                                    │
│  - robotPoseToField (机器人球场坐标)                          │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│                    Brain.tick()                               │
│  1. updateMemory() - 更新记忆数据                            │
│  2. handleSpecialStates() - 处理特殊状态                     │
│  3. handleCooperation() - 处理多机配合                       │
│  4. BrainTree.tick() - 执行行为树决策                        │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│                    RobotClient                                │
│  - setVelocity(vx, vy, vtheta)                               │
│  - moveHead(pitch, yaw)                                      │
│  - kickBall(speed, dir)                                      │
└───────────────────────────┬──────────────────────────────────┘
                            │
                            ▼
                    机器人执行动作
```

---

## 4. 核心类详解

### 4.1 Brain 类 (`include/brain.h`, `src/brain.cpp`)

**职责**: 核心决策类，整合所有子系统，实现比赛策略

#### 4.1.1 成员变量

```cpp
class Brain : public rclcpp::Node
{
    std::shared_ptr<BrainConfig> config;      // 静态配置
    std::shared_ptr<BrainLog> log;            // 日志系统
    std::shared_ptr<BrainData> data;          // 运行时数据
    std::shared_ptr<RobotClient> client;      // 机器人控制
    std::shared_ptr<Locator> locator;         // 定位系统
    std::shared_ptr<BrainTree> tree;          // 行为树
    std::shared_ptr<BrainCommunication> communication; // 通信
}
```

#### 4.1.2 核心方法

| 方法 | 功能 | 调用时机 |
|------|------|----------|
| `init()` | 初始化所有子系统 | 启动时调用一次 |
| `tick()` | 主决策循环 | 100Hz 循环调用 |
| `handleSpecialStates()` | 处理开球、任意球等特殊状态 | tick 中调用 |
| `handleCooperation()` | 处理多机配合逻辑 | tick 中调用 |
| `updateMemory()` | 更新记忆数据 (球、机器人等) | tick 中调用 |

#### 4.1.3 回调函数

```cpp
// 视觉检测回调 - 处理足球、机器人、场地标记识别
void detectionsCallback(const vision_interface::msg::Detections &msg);

// 场地线回调 - 处理场地标线识别
void fieldLineCallback(const vision_interface::msg::LineSegments &msg);

// 里程计回调 - 更新机器人里程计数据
void odometerCallback(const booster_interface::msg::Odometer &msg);

// 底层状态回调 - 处理 IMU、关节状态
void lowStateCallback(const booster_interface::msg::LowState &msg);

// 裁判数据回调 - 处理比赛状态
void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);

// 图像回调 - 记录图像到日志
void imageCallback(const sensor_msgs::msg::Image &msg);

// 深度图像回调 - 处理深度信息用于避障
void depthImageCallback(const sensor_msgs::msg::Image &msg);
```

#### 4.1.4 关键算法

**球位置预测** (`updateBallPrediction`):
- 使用历史数据拟合球的运动轨迹
- 预测未来位置用于拦截

**踢球角度计算** (`calcKickDir`):
- 计算机器人到球的角度
- 计算球到对方球门的角度
- 考虑门柱遮挡，选择最优射门方向

**威胁等级评估** (`threatLevel`):
- 0: 没看见敌人
- 1: 看见敌人但距离远
- 2: 看见敌人且距离近

### 4.2 BrainConfig 类 (`include/brain_config.h`, `src/brain_config.cpp`)

**职责**: 管理所有静态配置参数

#### 4.2.1 配置加载流程

```
config/config.yaml (默认值)
       +
config/config_local.yaml (本地覆盖，可选)
       +
launch.py 传入的参数 (运行时覆盖)
       │
       ▼
  BrainConfig::handle() - 参数验证和计算
       │
       ▼
  calcMapLines() - 计算场地标线理论位置
  calcMapMarkings() - 计算场地标记理论位置
```

#### 4.2.2 关键配置项

```yaml
# 比赛配置
game:
  team_id: 66              # 队伍 ID (必须修改)
  player_id: 3             # 球员 ID 1-5 (必须修改)
  player_role: "striker"   # 角色：striker | goal_keeper
  player_start_pos: "right" # 起始位置：left | right
  field_type: "adult_size" # 场地大小

# 机器人配置
robot:
  robot_height: 0.90       # 机器人身高 (米)
  odom_factor: 1.2         # 里程计修正系数
  vx_limit: 1.5            # X 方向速度上限
  vy_limit: 1.5            # Y 方向速度上限
  vtheta_limit: 1.5        # 旋转速度上限

# 策略配置
strategy:
  ball_confidence_threshold: 30.0  # 球置信度阈值
  ball_memory_timeout: 3.0         # 球记忆超时时间 (秒)
  enable_stable_kick: false        # 是否启用稳定踢球
  enable_shoot: false              # 是否启用射门
  enable_bypass: false             # 是否启用绕过

# 避障配置
obstacle_avoidance:
  collision_threshold: 0.3   # 碰撞阈值 (米)
  safe_distance: 1.0         # 安全距离 (米)
  avoid_secs: 2.0            # 避障时长 (秒)

# 定位配置
locator:
  min_marker_count: 3        # 最少标记数量
  max_residual: 0.25         # 最大残差

# 日志配置
rerunLog:
  enable_tcp: false          # 是否启用 TCP 传输
  server_ip: "192.168.10.110:9876"
  enable_file: false         # 是否启用文件记录
  log_dir: "/home/booster/Workspace/rrlog"
```

### 4.3 BrainData 类 (`include/brain_data.h`, `src/brain_data.cpp`)

**职责**: 管理所有运行时动态数据

#### 4.3.1 数据结构

```cpp
class BrainData
{
    // 比赛状态
    int score = 0;                    // 己方得分
    int oppoScore = 0;                // 对方得分
    int penalty[HL_MAX_NUM_PLAYERS];  // 罚球状态
    bool isKickingOff = false;        // 是否开球中
    string realGameSubState;          // 比赛子状态

    // 机器人位姿
    Pose2D robotPoseToOdom;    // 机器人在 Odom 坐标系
    Pose2D odomToField;        // Odom 在 Field 坐标系
    Pose2D robotPoseToField;   // 机器人在 Field 坐标系

    // 头部状态
    double headPitch;          // 头部 pitch 角度
    double headYaw;            // 头部 yaw 角度

    // 足球信息
    bool ballDetected = false;  // 是否检测到球
    GameObject ball;            // 球的详细信息
    GameObject tmBall;          // 队友传的球信息
    vector<array<double, 2>> predictedBallPos; // 预测球位置

    // 场地元素 (线程安全)
    vector<GameObject> getRobots();
    vector<GameObject> getGoalposts();
    vector<GameObject> getMarkings();
    vector<FieldLine> getFieldLines();
    vector<GameObject> getObstacles();

    // 运动规划
    double kickDir = 0.;        // 期望踢球方向
    string kickType = "shoot";  // 踢球类型

    // 多机配合
    TMStatus tmStatus[HL_MAX_NUM_PLAYERS]; // 队友状态
    int tmCmdId = 0;            // 全队指令 ID
    bool tmImLead = true;       // 我是否控球
    double tmMyCost = 0.;       // 我控球成本

    // 缓冲区
    Buffer<sensor_msgs::msg::Image> imageBuffer{50};
    Buffer<GameObject> detectionBuffer{50};
    Buffer<booster_interface::msg::Odometer> odomBuffer{50};
}
```

### 4.4 BrainTree 类 (`include/brain_tree.h`, `src/brain_tree.cpp`)

**职责**: 封装行为树引擎，管理决策逻辑

#### 4.4.1 行为树节点注册

```cpp
void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // 动作节点注册
    REGISTER_BUILDER(RobotFindBall)     // 机器人找球
    REGISTER_BUILDER(Chase)             // 追球
    REGISTER_BUILDER(Adjust)            // 调整位置
    REGISTER_BUILDER(Kick)              // 踢球
    REGISTER_BUILDER(CamTrackBall)      // 摄像头追踪球
    REGISTER_BUILDER(CamFindBall)       // 摄像头找球
    REGISTER_BUILDER(SelfLocate)        // 自定位
    REGISTER_BUILDER(MoveToPoseOnField) // 移动到目标位姿
    REGISTER_BUILDER(GoToFreekickPosition) // 任意球站位
    // ... 更多节点
}
```

#### 4.4.2 Blackboard 数据

行为树通过 Blackboard 共享数据：

```cpp
// 初始化 Blackboard 条目
void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<string>("decision", "");
    setEntry<string>("gc_game_state", "");
    setEntry<bool>("is_lead", true);
    // ...
}
```

### 4.5 RobotClient 类 (`include/robot_client.h`, `src/robot_client.cpp`)

**职责**: 封装所有机器人控制指令

#### 4.5.1 核心方法

| 方法 | 参数 | 功能 |
|------|------|------|
| `setVelocity(vx, vy, vtheta)` | 速度分量 | 设置机器人速度 |
| `moveHead(pitch, yaw)` | 头部角度 | 控制头部转动 |
| `kickBall(speed, dir)` | 速度、方向 | 踢球动作 |
| `standUp()` | - | 起身动作 |
| `squatBlock(side)` | "left"/"right" | 守门员蹲下 blocking |
| `crabWalk(angle, speed)` | 角度、速度 | 斜向行走 |
| `moveToPoseOnField(...)` | 目标位姿、参数 | 移动到目标位置 |

#### 4.5.2 速度控制实现

```cpp
int RobotClient::setVelocity(double x, double y, double theta, ...)
{
    // 1. 应用最小速度 (防止不响应)
    double minx = 0.05, miny = 0.08, mintheta = 0.05;
    if (fabs(x) < minx && fabs(x) > 1e-5) x = x > 0 ? minx : -minx;

    // 2. 限制速度上限
    x = cap(x, brain->config->vxLimit, -brain->config->vxLimit);
    y = cap(y, brain->config->vyLimit, -brain->config->vyLimit);
    theta = cap(theta, brain->config->vthetaLimit, -brain->config->vthetaLimit);

    // 3. 记录最后命令
    _vx = x; _vy = y; _vtheta = theta;
    _lastCmdTime = brain->get_clock()->now();

    // 4. 发送命令
    return call(booster_interface::CreateMoveMsg(x, y, theta));
}
```

### 4.6 Locator 类 (`include/locator.h`, `src/locator.cpp`)

**职责**: 粒子滤波定位

#### 4.6.1 定位流程

```
1. 初始化粒子 (genInitialParticles)
   - 在约束范围内随机生成粒子

2. 计算概率 (calcProbs)
   - 对每个粒子计算残差
   - 根据残差计算概率分布

3. 粒子重采样 (genParticles)
   - 根据概率分布采样新粒子
   - 添加高斯噪声

4. 收敛检查 (isConverged)
   - 检查粒子分布是否收敛
   - 收敛且残差合理则定位成功
```

#### 4.6.2 关键算法

**残差计算**:
```cpp
double Locator::residual(vector<FieldMarker> markers_r, Pose2D pose)
{
    double res = 0;
    for (int i = 0; i < markers_r.size(); i++) {
        auto marker_r = markers_r[i];
        auto marker_f = markerToFieldFrame(marker_r, pose);
        double dist = max(norm(marker_r.x, marker_r.y), 0.1);
        res += minDist(marker_f) / dist * 3; // 加权
    }
    return res;
}
```

**粒子更新**:
```cpp
// 根据概率密度函数计算粒子权重
for (int i = 0; i < rows; i++) {
    hypos(i, 4) = probDesity(hypos(i, 3), mu, sigma);
}

// 归一化
double probSum = hypos.col(4).sum();
hypos.col(4) = hypos.col(4) / probSum;
```

---

## 5. 数据流与回调函数

### 5.1 视觉数据处理流程

```
/detections (vision_interface::msg::Detections)
     │
     ▼
detectionsCallback()
     │
     ├─► getGameObjects() - 转换检测结果
     │      │
     │      ├─► detectProcessBalls() - 处理足球
     │      │      └─► updateRelativePos() - 更新相对位置
     │      │
     │      ├─► detectProcessMarkings() - 处理场地标记
     │      │      └─► identifyMarking() - 识别标记类型
     │      │
     │      ├─► detectProcessRobots() - 处理机器人
     │      │      └─► identifyGoalpost() - 识别球门柱
     │      │
     │      └─► detectProcessVisionBox() - 处理视野信息
     │
     └─► logDetection() - 记录到 Rerun 日志
```

### 5.2 球场坐标系转换

```
机器人坐标系 (robot)
    │
    │ posToRobot (检测得到)
    ▼
Odom 坐标系 (odom)
    │
    │ robotPoseToOdom (里程计积分)
    ▼
球场坐标系 (field)
    │
    │ robotPoseToField = robotPoseToOdom + odomToField
    │ odomToField 通过定位校准
    ▼
世界坐标系
```

### 5.3 内存更新机制

```cpp
void Brain::updateMemory()
{
    updateBallMemory();      // 更新球记忆
    updateRobotMemory();     // 更新机器人记忆
    updateObstacleMemory();  // 更新障碍物记忆
    updateKickoffMemory();   // 更新开球记忆
}

// 球记忆更新逻辑
void Brain::updateBallMemory()
{
    if (!ballDetected) {
        // 看不见球时，利用记忆中球的位置和机器人 Odom 更新
        if (msecsSince(ball.timePoint) > config->ballMemoryTimeout * 1000) {
            tree->setEntry<bool>("ball_location_known", false);
        }
    }
}
```

---

## 6. 行为树系统

### 6.1 主行为树结构 (`game.xml`)

```xml
MainTree (根)
│
├─ RunOnce: 初始化控制状态
│
├─ ReactiveSequence [control_state==1 || assist_kick || go_manual || assist_chase]
│   └─ 手动控制模式
│
├─ ReactiveSequence [control_state==2]
│   └─ 定位模式 (需要 pickup 时使用)
│
└─ ReactiveSequence [control_state==3 && !go_manual]
    │
    ├─ AutoGetUpAndLocate (自动起身定位)
    │
    ├─ ReactiveSequence [!gc_is_under_penalty]
    │   │
    │   ├─ [gc_game_sub_state_type=='TIMEOUT']
    │   │   └─ 暂停状态
    │   │
    │   ├─ [gc_game_sub_state_type=='NONE']
    │   │   │
    │   │   ├─ [gc_game_state=='INITIAL']
    │   │   │   └─ 入场位置站好
    │   │   │
    │   │   ├─ [gc_game_state=='READY']
    │   │   │   └─ 走到场内起始位置
    │   │   │
    │   │   ├─ [gc_game_state=='SET']
    │   │   │   └─ 站好等待比赛开始
    │   │   │
    │   │   └─ [gc_game_state=='PLAY']
    │   │       ├─ StrikerPlay (前锋比赛逻辑)
    │   │       └─ GoalKeeperPlay (守门员比赛逻辑)
    │   │
    │   └─ [gc_game_sub_state_type=='FREE_KICK']
    │       ├─ [gc_game_sub_state=='STOP']
    │       ├─ [gc_game_sub_state=='GET_READY']
    │       │   ├─ StrikerFreekick (前锋任意球)
    │       │   └─ GoalKeeperFreekick (守门员任意球)
    │       └─ [gc_game_sub_state=='SET']
```

### 6.2 前锋行为树 (`subtree_striker_play.xml`)

```
StrikerPlay
│
├─ SelfLocate (自定位)
│
├─ Locate (定位子树)
│
└─ ReactiveSequence [!ball_out]
    │
    ├─ CamFindAndTrackBall (摄像头找球追踪)
    │
    └─ ReactiveSequence
        │
        ├─ CalcKickDir (计算踢球方向)
        │
        └─ Sequence
            │
            ├─ StrikerDecide (前锋决策)
            │   └─ 输出 decision: find | chase | adjust | kick | cross | assist
            │
            ├─ FindBall [decision=='find'] (找球)
            │
            ├─ Assist [decision=='assist'] (协助)
            │
            ├─ Chase [decision=='chase'] (追球)
            │
            ├─ Adjust [decision=='adjust'] (调整位置)
            │
            ├─ Kick [decision=='kick'] (踢球)
            │
            └─ Cross [decision=='cross'] (横传)
```

### 6.3 关键行为节点实现

#### 6.3.1 CamTrackBall (摄像头追踪球)

```cpp
NodeStatus CamTrackBall::tick()
{
    // 1. 检查是否知道球的位置
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable)) return SUCCESS;

    // 2. 如果没看见球，看向记忆中球的位置
    if (!iSeeBall) {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else { // 看见了，追踪球
        ballX = mean(ball.boundingBox.xmax, ball.boundingBox.xmin);
        ballY = mean(ball.boundingBox.ymax, ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter;

        // 在中心容差范围内？
        if (fabs(deltaX) < pixToleranceX && fabs(deltaY) < pixToleranceY) {
            return SUCCESS; // 已在中心
        }

        // 计算头部运动
        double smoother = 3.5;
        double deltaYaw = deltaX / camPixX * camAngleX / smoother;
        double deltaPitch = deltaY / camPixY * camAngleY / smoother;
        pitch = headPitch + deltaPitch;
        yaw = headYaw - deltaYaw;
    }

    // 3. 发送头部命令
    brain->client->moveHead(pitch, yaw);
    return SUCCESS;
}
```

#### 6.3.2 Chase (追球)

```cpp
NodeStatus Chase::tick()
{
    double vx, vy, vtheta;
    double ballX = brain->data->ball.posToRobot.x;
    double ballY = brain->data->ball.posToRobot.y;

    // 1. 计算到球的角度和距离
    double targetAngle = atan2(ballY, ballX);
    double targetDist = norm(ballX, ballY);

    // 2. 状态机
    if (_state == "turn") {
        // 转向球
        vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        vx = vy = 0;
        if (fabs(targetAngle) < turn_threshold) _state = "approach";
    }
    else if (_state == "approach") {
        // 接近球
        vx = cap(ballX, vxLimit, -vxLimit);
        vy = cap(ballY, vyLimit, -vyLimit);
        vtheta = 0;
        if (targetDist < dist) _state = "align";
    }
    else if (_state == "align") {
        // 对齐
        // ...
    }

    brain->client->setVelocity(vx, vy, vtheta);
    return SUCCESS;
}
```

#### 6.3.3 Kick (踢球)

```cpp
NodeStatus Kick::onStart()
{
    _startTime = brain->get_clock()->now();
    _state = "stabilize";
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    double elapsed = msecsSince(_startTime);

    if (_state == "stabilize") {
        // 稳定阶段：停止运动，准备踢球
        brain->client->setVelocity(0, 0, 0);
        if (elapsed >= _msecsStablize) {
            _state = "kick";
            _startTime = brain->get_clock()->now();
        }
    }
    else if (_state == "kick") {
        // 踢球阶段
        auto [speed, dir, cancel] = _calcSpeed();
        brain->client->kickBall(speed, dir, cancel);

        if (elapsed >= _msecKick) {
            return NodeStatus::SUCCESS;
        }
    }
    return NodeStatus::RUNNING;
}
```

---

## 7. 配置文件详解

### 7.1 配置加载顺序

```python
# launch.py 中的配置加载顺序
parameters=[
    config_file,      # config/config.yaml (基础配置)
    config_local_file, # config/config_local.yaml (本地覆盖，可选)
    config            # launch 时传入的参数 (运行时覆盖)
]
```

### 7.2 关键配置说明

#### 比赛配置 (`game`)

```yaml
game:
  team_id: 66
  # 队伍 ID，必须与裁判机配置一致
  # 有效值：1-255

  player_id: 3
  # 球员 ID，场上唯一
  # 有效值：1 | 2 | 3 | 4 | 5

  player_role: "striker"
  # 球员角色
  # striker: 前锋，负责进攻和控球
  # goal_keeper: 守门员，负责防守球门

  player_start_pos: "right"
  # 开局站位
  # left: 左侧 | right: 右侧

  field_type: "adult_size"
  # 场地规格
  # adult_size: 14.16m x 9.22m (标准成人场)
  # kid_size: 9m x 6m (儿童场)
  # robo_league: 22.003m x 14.126m (RoboLeague 场)

  treat_person_as_robot: false
  # 调试用，是否将人类视为机器人
  # 正式比赛必须为 false
```

#### 机器人配置 (`robot`)

```yaml
robot:
  robot_height: 0.90
  # 机器人身高 (米)，用于视觉测距

  odom_factor: 1.2
  # 里程计修正系数
  # 实际距离 = 里程计读数 × odom_factor

  vx_factor: 0.90
  # X 方向速度修正，补偿实际速度比指令大的问题

  yaw_offset: 0.0
  # Yaw 方向偏移修正，补偿定位角度偏差

  vx_limit: 1.5
  vy_limit: 1.5
  vtheta_limit: 1.5
  # 各方向速度上限 (m/s, rad/s)
```

#### 策略配置 (`strategy`)

```yaml
strategy:
  ball_confidence_threshold: 30.0
  # 置信度阈值，低于此值认为不是球
  # detection 模块传入的置信度都 > 20

  ball_confidence_decay_rate: 3.0
  # 置信度衰减速率
  # 看到 confidence=100 的球后，多少秒没再看到则认为球丢了

  ball_memory_timeout: 3.0
  # 球记忆超时时间 (秒)
  # 连续多少秒看不到球，认为球的位置不可信

  tm_ball_dist_threshold: 1.5
  # 队友传来的球位置与我的距离阈值 (米)
  # 大于此值才信任队友的球位置信息

  limit_near_ball_speed: true
  near_ball_speed_limit: 0.6
  near_ball_range: 2.0
  # 靠近球时的速度限制
  # 在 2.0 米范围内，速度限制为 0.6 m/s

  enable_stable_kick: false
  # 是否启用稳定踢球
  # 开启后，踢球前会先稳定一下

  enable_shoot: false
  # 是否启用射门
  # 真射门动作，力量较大

  enable_directional_kick: false
  # 是否启用方向性踢球
  # 可以指定踢球方向

  enable_bypass: false
  # 是否启用绕过
  # 绕过障碍物或对方机器人
```

#### 渐进式踢球配置 (`strategy.progressive_kick`)

```yaml
strategy:
  progressive_kick:
    enable: true
    # 是否启用渐进式踢球策略

    dribble:
      speed: 0.3      # 慢带速度 (m/s)
      adjust_interval: 100  # 方向调整间隔 (ms)
      max_dist: 0.5   # 最大带球距离 (m)

    power_kick:
      speed: 1.5      # 趟球速度 (m/s)
      min_msec: 300   # 最小踢球时间 (ms)
```

#### 射门配置 (`strategy.shoot`)

```yaml
strategy:
  shoot:
    threat_threshold: -2.0
    # 威胁阈值，低于此值认为有射门机会

    xmin: 0.3
    xmax: 0.6
    ymin: -0.3
    ymax: 0.3
    # 射门区域范围 (相对于球)
```

#### 避障配置 (`obstacle_avoidance`)

```yaml
obstacle_avoidance:
  depth_sample_step: 16
  # 深度采样步长

  obstacle_min_height: 0.3
  # 障碍物最小高度 (米)

  grid_size: 0.2
  # 栅格大小 (米)

  max_x: 3.0
  max_y: 5.0
  # 检测范围 (米)

  exclusion_x: 0.4
  exclusion_y: 0.5
  # 机器人自身排除区域

  occupancy_threshold: 8
  # 占用阈值，高于此值认为是障碍物

  collision_threshold: 0.3
  # 碰撞阈值 (米)

  safe_distance: 1.0
  # 安全距离 (米)

  avoid_secs: 2.0
  # 避障时长 (秒)

  enable_freekick_avoid: true
  # 是否在任意球时避障

  obstacle_memory_msecs: 500
  # 障碍物记忆时间 (毫秒)
```

#### 定位配置 (`locator`)

```yaml
locator:
  min_marker_count: 3
  # 最少需要看到的标记数量
  # 低于此值无法定位

  max_residual: 0.25
  # 最大允许残差
  # 定位结果残差高于此值认为定位失败
```

#### 日志配置 (`rerunLog`)

```yaml
rerunLog:
  enable_tcp: false
  # 是否启用 TCP 传输到 Rerun Viewer

  server_ip: "192.168.10.110:9876"
  # Rerun Viewer 服务器地址

  enable_file: false
  # 是否启用本地文件记录

  log_dir: "/home/booster/Workspace/rrlog"
  # 本地日志存储路径

  max_log_file_mins: 5.0
  # 单个日志文件最大时长 (分钟)
  # 太大文件会导致读取时死机

  img_interval: 1
  # 每多少帧记录一次图像
  # 调高可减少日志大小，但降低时间精度
```

---

## 8. 通信系统

### 8.1 通信架构

```
┌─────────────────────────────────────────────────────────────┐
│                    BrainCommunication                        │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              裁判通信 (GameController)               │   │
│  │  UDP 单播 - 发送存活状态                            │   │
│  │  Port: GAMECONTROLLER_RETURN_PORT                   │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              发现通信 (Discovery)                    │   │
│  │  UDP 广播 - 发现队友                                 │   │
│  │  Port: 20000 + teamId                               │   │
│  │  Interval: 1000ms                                   │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              队内通信 (Team Communication)           │   │
│  │  UDP 单播 - 共享状态信息                            │   │
│  │  Port: 30000 + teamId                               │   │
│  │  Interval: 100ms                                    │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 通信消息结构

#### 发现消息 (`TeamDiscoveryMsg`)

```cpp
struct TeamDiscoveryMsg {
    int communicationId;    // 通信 ID
    int teamId;             // 队伍 ID
    int playerId;           // 球员 ID
    string playerRole;      // 球员角色
    bool isAlive;           // 是否存活
    bool ballDetected;      // 是否检测到球
    double ballRange;       // 球的距离
    Point ballPosToField;   // 球在球场的位置
    Pose2D robotPoseToField; // 机器人在球场的位置
    int validation;         // 验证字段
};
```

#### 队内通信消息 (`TeamCommunicationMsg`)

```cpp
struct TeamCommunicationMsg {
    int validation;         // 验证字段
    int communicationId;    // 通信 ID
    int teamId;             // 队伍 ID
    int playerId;           // 球员 ID
    int playerRole;         // 1=striker, 2=goal_keeper
    bool isAlive;           // 是否存活
    bool isLead;            // 是否控球
    bool ballDetected;      // 是否检测到球
    bool ballLocationKnown; // 是否知道球位置
    double ballConfidence;  // 球置信度
    double ballRange;       // 球的距离
    double cost;            // 控球成本
    Point ballPosToField;   // 球在球场的位置
    Pose2D robotPoseToField; // 机器人在球场的位置
    double kickDir;         // 期望踢球方向
    double thetaRb;         // 机器人到球的角度
    int cmdId;              // 指令 ID
    int cmd;                // 指令内容
};
```

### 8.3 多机配合逻辑

```cpp
void Brain::handleCooperation()
{
    // 1. 收集存活的队友
    vector<int> aliveTmIdxs = [];
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (data->penalty[i] != PENALTY_NONE) continue;
        if (msecsSince(data->tmStatus[i].timeLastCom) > COM_TIMEOUT) continue;
        aliveTmIdxs.push_back(i);
    }

    // 2. 查找可信的队友球信息
    int trustedTMIdx = -1;
    double minRange = 1e6;
    for (int tmIdx : aliveTmIdxs) {
        auto status = data->tmStatus[tmIdx];
        if (status.ballDetected && status.ballRange < minRange) {
            double dist = norm(status.ballPosToField - myPos);
            if (dist > config->tmBallDistThreshold) {
                trustedTMIdx = tmIdx;
                minRange = status.ballRange;
            }
        }
    }

    // 3. 使用可信的球信息
    if (trustedTMIdx >= 0) {
        data->tmBall = data->tmStatus[trustedTMIdx].ball;
        if (!ball_location_known) {
            data->ball = data->tmBall;
        }
    }

    // 4. 角色切换逻辑
    if (config->enable_role_switch) {
        // 根据成本和状态决定是否切换角色
    }
}
```

---

## 9. 定位系统

### 9.1 定位算法概述

采用**粒子滤波**算法进行定位：

1. **粒子初始化**: 在约束范围内随机生成粒子
2. **粒子更新**: 根据观测数据更新粒子权重
3. **粒子重采样**: 根据权重重新采样粒子
4. **收敛判断**: 检查粒子分布是否收敛

### 9.2 场地标记

```cpp
void Locator::calcFieldMarkers()
{
    // 中线上的 X 标志
    fieldMarkers.push_back({'X', 0.0, -circleRadius});
    fieldMarkers.push_back({'X', 0.0, circleRadius});

    // 罚球点
    fieldMarkers.push_back({'P', length/2 - penaltyDist, 0.0});
    fieldMarkers.push_back({'P', -length/2 + penaltyDist, 0.0});

    // 边线中心
    fieldMarkers.push_back({'T', 0.0, width/2});
    fieldMarkers.push_back({'T', 0.0, -width/2});

    // 禁区角点
    // 球门区角点
    // 场地四角
}
```

### 9.3 定位模式

根据看到的标记类型，支持多种定位模式：

| 模式 | 标识 | 说明 |
|------|------|------|
| SelfLocate1P | 1P | 看到一个罚球点 |
| SelfLocate1M | 1M | 看到一个边线中点 |
| SelfLocate2X | 2X | 看到两个 X 标志 |
| SelfLocate2T | 2T | 看到两个 T 标志 |
| SelfLocateLT | LT | 看到 L 和 T 标志 |
| SelfLocatePT | PT | 看到 P 和 T 标志 |
| SelfLocateBorder | Border | 看到边界标记 |

---

## 10. 机器人控制

### 10.1 运动控制

#### 10.1.1 速度控制

```cpp
// 基本速度控制
client->setVelocity(vx, vy, vtheta);

// 参数含义:
// vx: X 方向速度 (前后)，向前为正
// vy: Y 方向速度 (左右)，向左为正
// vtheta: 旋转速度 (逆时针为正)
```

#### 10.1.2 螃蟹行走 (斜向移动)

```cpp
// 向指定角度斜向行走
client->crabWalk(angle, speed);

// angle: 移动方向 (相对于机器人前方)
// speed: 移动速度
```

#### 10.1.3 移动到目标位姿

```cpp
// 版本 1: 基础版
client->moveToPoseOnField(
    tx, ty, ttheta,           // 目标位置和角度
    longRangeThreshold,       // 远距阈值
    turnThreshold,            // 转向阈值
    vxLimit, vyLimit, vthetaLimit, // 速度限制
    xTolerance, yTolerance, thetaTolerance, // 到达容差
    avoidObstacle             // 是否避障
);
```

### 10.2 避障逻辑

```cpp
// 1. 构建深度栅格地图
vector<vector<int>> grid;
for (depth_sample : depth_image) {
    if (depth < obstacle_min_height) continue;
    grid[x][y]++;
}

// 2. 检测碰撞
double distToObstacle(double angle) {
    for (dist = 0; dist < max_range; dist += step) {
        x = dist * cos(angle);
        y = dist * sin(angle);
        if (grid[x][y] > occupancy_threshold) {
            return dist;
        }
    }
    return max_range;
}

// 3. 避障决策
if (distToObstacle(0) < safe_distance) {
    // 需要避障
    double avoidDir = calcAvoidDir(targetAngle, safe_distance);
    vtheta = avoidDir * vthetaLimit;
}
```

### 10.3 头部控制

```cpp
// 头部运动范围限制
// Pitch: 0.45 (向上) ~ 2.0 (向下) rad
// Yaw: -1.1 (向右) ~ 1.1 (向左) rad

client->moveHead(pitch, yaw);

// 软限位检查
yaw = cap(yaw, headYawLimitLeft, headYawLimitRight);
pitch = max(pitch, headPitchLimitUp);
```

### 10.4 踢球动作

```cpp
// 基本踢球
client->kickBall(kick_speed, kick_dir, cancel);

// RL 视觉踢球 (已移除)
client->RLVisionKick();

// 花式踢球
client->fancyKickBall(kick_speed, kick_dir, cancel);

// 守门员蹲下 blocking
client->squatBlock("left" | "right");
client->squatUp();
```

---

## 附录

### A. 启动命令

```bash
# 基本启动
ros2 launch brain launch.py

# 指定角色和位置
ros2 launch brain launch.py role:=striker pos:=left

# 仿真模式
ros2 launch brain launch.py sim:=true

# 禁用日志
ros2 launch brain launch.py disable_log:=true

# 禁用通信
ros2 launch brain launch.py disable_com:=true

# 指定行为树
ros2 launch brain launch.py tree:=game.xml
```

### B. 调试工具

```bash
# 查看节点
ros2 node list

# 查看话题
ros2 topic list

# 查看参数
ros2 param dump /brain_node

# 查看日志
tail -f ~/.ros/log/latest/brain_node*.log
```

### C. 关键话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/booster_vision/detection` | Detections | 输入 | 视觉检测结果 |
| `/booster_vision/line_segments` | LineSegments | 输入 | 场地线检测结果 |
| `/odometer_state` | Odometer | 输入 | 里程计数据 |
| `/low_state` | LowState | 输入 | 底层状态 (IMU、关节) |
| `/remote_controller_state` | RemoteControllerState | 输入 | 遥控器状态 |
| `/robocup/game_controller` | GameControlData | 输入 | 裁判数据 |
| `/LocoApiTopicReq` | RpcReqMsg | 输出 | 机器人控制指令 |
| `/play_sound` | String | 输出 | 声音播放 |
| `/speak` | String | 输出 | TTS 语音 |

### D. 状态机

```
比赛状态流转:
INITIAL → READY → SET → PLAY → END

任意球状态流转:
STOP → GET_READY → SET → PLAY
```

---

*文档生成时间：2026-03-27*
*基于代码版本：test 分支*
