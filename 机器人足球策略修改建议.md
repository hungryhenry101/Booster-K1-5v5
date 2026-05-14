# 机器人足球策略修改建议

本文档提供了一系列改进机器人配合的策略建议，每个建议都包含具体的代码实现说明。

其中关于边线开球时，机器人一直大力射门，在第15处修改中给出了修改样例，可以参考修改使用。

由于备赛时间紧张，建议选择其中一个或几个例子来做修改，并且比赛前充分调试。

---

## 1. 引入多角色体系

### 建议说明

在现有的 `striker` 和 `goal_keeper` 基础上，细化为 5 种角色：

- **PRIMARY_STRIKER（主攻手）**：负责主要进攻
- **SUPPORT_STRIKER（辅助前锋）**：协助主攻手，准备接应
- **MIDFIELDER（中场）**：连接攻防
- **DEFENDER（后卫）**：主要负责防守
- **GOAL_KEEPER（守门员）**：守门

### 代码实现

**修改位置**: `src/brain/include/types.h`

在文件中添加角色枚举和球权状态枚举：

```cpp
// 在文件适当位置添加
enum class PlayerRole {
    PRIMARY_STRIKER = 0,
    SUPPORT_STRIKER = 1,
    MIDFIELDER = 2,
    DEFENDER = 3,
    GOAL_KEEPER = 4,
    UNKNOWN = 5
};

enum class BallPossession {
    OURS,
    THEIRS,
    LOOSE
};

enum class RobotIntention {
    CHASING,
    KICKING,
    DEFENDING,
    SUPPORTING,
    POSITIONING,
    IDLE
};
```

**修改位置**: `src/brain/include/brain_data.h`

在 `BrainData` 类中添加新字段：

```cpp
// 在 BrainData 类的成员变量区域添加
int passTargetPlayerId = -1; // 传球目标队友的 playerId, -1 表示无目标

// 球权状态
BallPossession ballPossession = BallPossession::LOOSE;
rclcpp::Time ballPossessionLastUpdate;

// 球速度估计
double ballVelX = 0.;
double ballVelY = 0.;
rclcpp::Time ballVelLastTime;
Point ballVelLastPos;

// 机器人速度
double robotVelX = 0.;
double robotVelY = 0.;

// 当前意图
RobotIntention myIntention = RobotIntention::IDLE;

// 拖延策略
bool enableTimeWasting = false;
rclcpp::Time timeWastingStartTime;
double timeWastingDuration = 30.0;

// Adjust 超时
rclcpp::Time adjustStartTime;
bool adjustTimerActive = false;
double adjustTimeoutSecs = 5.0;

// 多角色体系
PlayerRole myPlayerRole = PlayerRole::UNKNOWN;
```

在 `TMStatus` 结构体中添加：

```cpp
struct TMStatus {
    string role = "not initialized";
    PlayerRole playerRole = PlayerRole::UNKNOWN;
    bool isAlive = false;
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.;
    double ballRange = 0.;
    double cost = 0.;
    bool isLead = true;
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir = 0.;
    double thetaRb = 0.;
    int cmd = 0;
    int cmdId = 0;
    rclcpp::Time timeLastCom;
    double ballVelX = 0.;
    double ballVelY = 0.;
    double robotVelX = 0.;
    double robotVelY = 0.;
    RobotIntention intention = RobotIntention::IDLE;
    int intentionInt = 0;
};
```

---

## 2. 改进 Assist 接应位

### 建议说明

让 `cost` 排名 1 的机器人在进攻时前插到球前方接应，而不是只在球后方防守。当场上有 3 名以上队友时，采用更积极的前插策略。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `Assist::tick()` 函数

```cpp
NodeStatus Assist::tick() {
    // ... 现有代码 ...

    double kickDir = brain->data->kickDir;
    double defenseDir = atan2(ballPos.y, ballPos.x + fd.length / 2.0);

    if (brain->data->tmMyCostRank == 1) {
        if (aggressiveAssist) {
            // 前插接应：在球前方 2.5 米处，沿踢球方向
            double supportDist = 2.5;
            targetPose.x = ballPos.x + supportDist * cos(kickDir);
            targetPose.y = ballPos.y + supportDist * sin(kickDir);

            // 限制不要超出禁区
            if (targetPose.x > oppGoalX - fd.goalAreaLength - 0.3) {
                targetPose.x = oppGoalX - fd.goalAreaLength - 0.3;
            }
            if (fabs(targetPose.y) > fd.width / 2.0 - 1.0) {
                targetPose.y = cap(targetPose.y, fd.width / 2.0 - 1.0, -fd.width / 2.0 + 1.0);
            }
            targetPose.theta = kickDir;
        } else {
            // 保守防守位置
            targetPose.x = ballPos.x - 2.0;
            targetPose.x = max(targetPose.x, -fd.length / 2.0 + distToGoalline);
            targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0);
        }
    }

    // ... 后续代码 ...
}
```

---

## 3. 增加传球机制

### 建议说明

在 `CalcKickDir` 中增加传球决策逻辑，当射门角度不好时，寻找最佳传球目标。在 `StrikerDecide` 中增加 `pass` 决策分支。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `CalcKickDir::tick()` 函数

在函数开始处添加传球逻辑：

```cpp
NodeStatus CalcKickDir::tick()
{
    // ... 现有代码 ...

    bool angleGoodForShoot = brain->isAngleGood(0.3, "shoot");
    bool enablePass;
    brain->get_parameter("strategy.enable_pass", enablePass);

    brain->data->passTargetPlayerId = -1;

    if (!angleGoodForShoot && enablePass) {
        int bestTmId = -1;
        double bestTmScore = -1e6;
        int selfIdx = brain->config->playerId - 1;

        // 遍历所有队友，寻找最佳传球目标
        for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
            if (i == selfIdx) continue;
            auto tmStatus = brain->data->tmStatus[i];
            if (!tmStatus.isAlive) continue;
            if (tmStatus.role == "goal_keeper") continue;

            auto tmPos = tmStatus.robotPoseToField;
            double tmDistToBall = norm(tmPos.x - bPos.x, tmPos.y - bPos.y);
            if (tmDistToBall < 1.0) continue; // 太近不传球

            double passDir = atan2(tmPos.y - bPos.y, tmPos.x - bPos.x);
            double dirToGoal = atan2(-bPos.y, fd.length / 2.0 - bPos.x);
            double angleToGoal = fabs(toPInPI(passDir - dirToGoal));

            // 评分：位置越靠前越好，角度越小越好，距离适中
            double score = tmPos.x - angleToGoal * 2.0 - tmDistToBall * 0.5;
            if (tmPos.x > bPos.x) score += 3.0; // 在球前方加分

            if (score > bestTmScore) {
                bestTmScore = score;
                bestTmId = i;
            }
        }

        if (bestTmId >= 0) {
            auto tmPos = brain->data->tmStatus[bestTmId].robotPoseToField;
            double passDir = atan2(tmPos.y - bPos.y, tmPos.x - bPos.x);
            double passDist = norm(tmPos.x - bPos.x, tmPos.y - bPos.y);

            // 检查传球路径是否有障碍
            bool passPathClear = true;
            auto obstacles = brain->data->getObstacles();
            for (auto& obs : obstacles) {
                double obsDist = pointMinDistToLine(
                    Point2D{obs.posToField.x, obs.posToField.y},
                    Line{bPos.x, bPos.y, tmPos.x, tmPos.y}
                );
                if (obsDist < 0.5) {
                    passPathClear = false;
                    break;
                }
            }

            if (passPathClear && passDist < 8.0 && passDist > 1.5) {
                brain->data->kickType = "pass";
                brain->data->kickDir = passDir;
                brain->data->passTargetPlayerId = bestTmId + 1; // playerId 从 1 开始
                return NodeStatus::SUCCESS;
            }
        }
    }

    // ... 后续原有的 cross/shoot/block 逻辑 ...
}
```

**修改位置**: `src/brain/src/brain_tree.cpp` - `StrikerDecide::tick()` 函数

在决策分支中添加 `pass`：

```cpp
if (brain->data->kickType == "cross") newDecision = "cross";
else if (brain->data->kickType == "pass") newDecision = "pass";
else {
    // ... 原有的 kick/shoot 逻辑 ...
}
```

**修改位置**: `src/brain/behavior_trees/subtrees/subtree_striker_play.xml`

添加 `pass` 的 `Kick` 节点：

```xml
<Kick _while="decision == 'pass'" speed_limit="0.7" min_msec_kick="800"/>
```

---

## 4. 改进 cross 判断

### 建议说明

在判断是否传中时，检查禁区内是否有队友，避免盲目传中。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `CalcKickDir::tick()` 函数

在 `cross` 判断前添加队友位置检查：

```cpp
bool hasTeammateInBox = false;
if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
    int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue;
        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue;

        auto tmPos = tmStatus.robotPoseToField;
        // 检查是否在对方禁区内
        if (tmPos.x > fd.length / 2.0 - fd.penaltyAreaLength - 1.0
            && fabs(tmPos.y) < fd.penaltyAreaWidth / 2.0 + 1.0) {
            hasTeammateInBox = true;
            break;
        }
    }
}

// 只有在禁区内有队友时才传中
if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius && hasTeammateInBox) {
    brain->data->kickType = "cross";
    // ...
}
```

---

## 5. 优化守门员预测站位

### 建议说明

守门员根据球速预测球的落点，提前移动到预测位置，而不是只根据当前球的位置站位。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `GoToGoalBlockingPosition::tick()` 函数

```cpp
NodeStatus GoToGoalBlockingPosition::tick() {
    // ... 现有代码 ...

    // 获取球速
    double ballVelX = brain->data->ballVelX;
    double ballVelY = brain->data->ballVelY;
    double ballSpeed = norm(ballVelX, ballVelY);

    // 预测球的未来位置
    double predictTime = 0.0;
    if (ballSpeed > 0.5) {
        predictTime = min(1.0, (ballPos.x + fd.length / 2.0) / max(ballVelX, -0.1));
        predictTime = max(predictTime, 0.0);
    }

    Point predictedBallPos;
    predictedBallPos.x = ballPos.x + ballVelX * predictTime;
    predictedBallPos.y = ballPos.y + ballVelY * predictTime;
    // 限制在场地范围内
    predictedBallPos.x = cap(predictedBallPos.x, -fd.length / 2.0, fd.length / 2.0);
    predictedBallPos.y = cap(predictedBallPos.y, -fd.width / 2.0, fd.width / 2.0);

    // 决定是否使用预测位置
    bool usePrediction = (ballSpeed > 0.5 && ballVelX < -0.3);
    auto effectiveBallPos = usePrediction ? predictedBallPos : ballPos;

    // 使用预测的球位置计算站位
    Pose2D targetPose;
    targetPose.x = curRole == "striker" ?
        (std::max(-fd.length / 2.0 + distToGoalline, effectiveBallPos.x - 1.5))
        : (-fd.length / 2.0 + distToGoalline);

    if (effectiveBallPos.x + fd.length / 2.0 < distToGoalline) {
        targetPose.y = curRole == "striker" ?
            (effectiveBallPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
            : (effectiveBallPos.y > 0 ? fd.goalWidth / 4.0 : -fd.goalWidth / 4.0);
    } else {
        targetPose.y = effectiveBallPos.y * distToGoalline / (effectiveBallPos.x + fd.length / 2.0);
        targetPose.y = curRole == "striker" ?
            (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth / 2.0, -fd.penaltyAreaWidth / 2.0));
    }

    // 预测时提高移动速度
    if (usePrediction) {
        vyLimit = min(vyLimit, 1.2);
    }

    // ... 后续代码 ...
}
```

---

## 6. 守门员出击时通知队友回防

### 建议说明

当守门员离开禁区出击时，通知所有场上队友切换到防守意图，加强防守。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `GoalieDecide::tick()` 函数

```cpp
NodeStatus GoalieDecide::tick()
{
    // ... 现有代码 ...

    auto fd = brain->config->fieldDimensions;
    bool goalieRushing = false;

    // ... 决策逻辑 ...

    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
        // 判断是否在禁区附近出击
        if (brain->data->ball.posToField.x < -fd.length / 2.0 + fd.penaltyAreaLength + 0.5) {
            goalieRushing = true;
        }
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
        if (brain->data->ball.posToField.x < -fd.length / 2.0 + fd.penaltyAreaLength) {
            goalieRushing = true;
        }
    }

    // 出击时通知队友回防
    if (goalieRushing) {
        brain->data->myIntention = RobotIntention::DEFENDING;
        for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
            if (brain->data->tmStatus[i].isAlive && brain->data->tmStatus[i].role != "goal_keeper") {
                brain->data->tmStatus[i].intention = RobotIntention::DEFENDING;
            }
        }
    }

    // ... 后续代码 ...
}
```

---

## 7. 调整 Chase 速度参数和 Adjust 超时机制

### 建议说明

- **Chase 速度**：根据机器人到球的距离动态调整速度，远距离时更快，近距离时更慢以便精确控球
- **Adjust 超时**：增加 Adjust 状态超时机制，防止机器人在 Adjust 状态卡住超过 5 秒

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `Chase::tick()` 函数

```cpp
NodeStatus Chase::tick() {
    // ... 现有代码 ...

    // 根据距离动态调整速度
    double distToBall = brain->data->ball.range;
    double speedFactor = 1.0;

    if (distToBall > 2.0) {
        speedFactor = 1.2;  // 远距离时加速
    } else if (distToBall < 0.5) {
        speedFactor = 0.5;  // 近距离时减速
    }

    vx = vx * speedFactor;
    vy = vy * speedFactor;

    // ... 后续代码 ...
}
```

**修改位置**: `src/brain/src/brain_tree.cpp` - `Adjust::tick()` 函数

```cpp
NodeStatus Adjust::tick()
{
    // 超时机制
    auto now = brain->get_clock()->now();
    if (!brain->data->adjustTimerActive) {
        brain->data->adjustStartTime = now;
        brain->data->adjustTimerActive = true;
    }

    auto elapsed = std::chrono::duration<double>(now - brain->data->adjustStartTime).count();
    if (elapsed > brain->data->adjustTimeoutSecs) {
        // 超时，强制退出 Adjust 状态
        brain->tree->setEntry("decision", "chase");
        return NodeStatus::SUCCESS;
    }

    // ... 原有代码 ...
}
```

---

## 8. 优化 FindBall 策略

### 建议说明

当自己找不到球时，如果队友看到了球，使用队友提供的球位置信息，而不是盲目搜索。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - `FindBall::tick()` 函数

```cpp
NodeStatus FindBall::tick() {
    // ... 现有代码 ...

    // 检查是否有队友看到球
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (tmBallPosReliable) {
        // 使用队友提供的球位置
        Point tmBallPos;
        double minCost = 1e6;
        for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
            if (brain->data->tmStatus[i].ballLocationKnown) {
                double cost = brain->data->tmStatus[i].cost;
                if (cost < minCost) {
                    minCost = cost;
                    tmBallPos = brain->data->tmStatus[i].ballPosToField;
                }
            }
        }

        // 转向队友提供的球位置
        double dirToTmBall = atan2(tmBallPos.y - brain->data->robotPoseToField.y,
                                    tmBallPos.x - brain->data->robotPoseToField.x);
        double yawErr = toPInPI(dirToTmBall - brain->data->robotPoseToField.theta);

        if (fabs(yawErr) > 0.3) {
            brain->client->setVelocity(0, 0, yawErr * 2.0);
            return NodeStatus::RUNNING;
        }
    }

    // ... 原有搜索逻辑 ...
}
```

---

## 9. 丰富团队通信

### 建议说明

在团队通信中增加球速、机器人速度、意图等字段，让队友之间共享更多信息。

### 代码实现

**修改位置**: `src/brain/src/brain.cpp` - 通信数据发布函数

```cpp
void Brain::publishTeamData() {
    // ... 现有代码 ...

    // 填充新的通信字段
    brainData->tmStatus[playerId - 1].ballVelX = data->ballVelX;
    brainData->tmStatus[playerId - 1].ballVelY = data->ballVelY;
    brainData->tmStatus[playerId - 1].robotVelX = data->robotVelX;
    brainData->tmStatus[playerId - 1].robotVelY = data->robotVelY;
    brainData->tmStatus[playerId - 1].intention = data->myIntention;
    brainData->tmStatus[playerId - 1].intentionInt = static_cast<int>(data->myIntention);
    brainData->tmStatus[playerId - 1].playerRole = data->myPlayerRole;

    // ... 后续代码 ...
}
```

---

## 10. 改进 Cost 计算

### 建议说明

在计算机器人到球的 `cost` 时，不仅考虑距离，还考虑球速方向和场地位置因素。

### 代码实现

**修改位置**: `src/brain/src/brain.cpp` - Cost 计算函数

```cpp
double Brain::calcCost() {
    double dist = data->ball.range;
    double yawCost = fabs(data->ball.yawToRobot) * 0.5;
    double baseCost = dist + yawCost;

    // 考虑球速方向
    double ballVelX = data->ballVelX;
    double ballVelY = data->ballVelY;
    double ballSpeed = norm(ballVelX, ballVelY);

    if (ballSpeed > 0.3) {
        double ballDir = atan2(ballVelY, ballVelX);
        double robotDir = data->robotPoseToField.theta;
        double dirDiff = fabs(toPInPI(ballDir - robotDir));

        // 如果机器人面向球运动方向，cost 降低
        if (dirDiff < M_PI / 3) {
            baseCost *= 0.8;
        }
    }

    // 考虑场地位置（前场 cost 降低，鼓励进攻）
    double fieldPosFactor = 1.0;
    if (data->robotPoseToField.x > 0) {
        fieldPosFactor = 0.9;  // 前场
    } else if (data->robotPoseToField.x < -data->config->fieldDimensions.length / 4) {
        fieldPosFactor = 1.1;  // 后场
    }

    return baseCost * fieldPosFactor;
}
```

---

## 11. 更新 config.yaml 参数

### 建议说明

在配置文件中添加新的策略参数。

### 代码实现

**修改位置**: `src/brain/config/config.yaml`

```yaml
strategy:
  enable_pass: true
  enable_auto_visual_kick: true
  enable_auto_visual_defend: true
  auto_visual_kick_enable_dist_min: 0.2
  auto_visual_kick_enable_dist_max: 4.0
  auto_visual_kick_enable_angle: 0.8
  shoot:
    threat_threshold: 0.7
  power_shoot:
    enable: true
    use_for_kickoff: false
  enable_shoot: true
  enable_bypass: true
  enable_directional_kick: true
  kick_range: 1.0
  kick_theta_range: 1.5
  adjust_timeout_secs: 5.0
  time_wasting:
    enable: false
    duration: 30.0

obstacle_avoidance:
  avoid_during_chase: true
  chase_ao_safe_dist: 0.5
  avoid_during_kick: true
  kick_ao_safe_dist: 0.3
  kick_ao_use_shoot: false
```

---

## 12. 增加球权状态判断

### 建议说明

实时判断球权归属（我方/对方/无主），根据球权调整策略。

### 代码实现

**修改位置**: `src/brain/src/brain.cpp` - 球权判断函数

```cpp
void Brain::updateBallPossession() {
    auto now = get_clock()->now();
    double distToBall = data->ball.range;
    double myCost = calcCost();

    // 获取最小队友 cost
    double minTmCost = 1e6;
    double minOppCost = 1e6;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (data->tmStatus[i].isAlive) {
            if (data->tmStatus[i].cost < minTmCost) {
                minTmCost = data->tmStatus[i].cost;
            }
        }
    }

    // 判断球权
    BallPossession newPossession;
    if (myCost < 3.0 && myCost <= minTmCost) {
        newPossession = BallPossession::OURS;
    } else if (minTmCost < 3.0 && minTmCost < minOppCost) {
        newPossession = BallPossession::OURS;
    } else if (minOppCost < 3.0 && minOppCost < minTmCost) {
        newPossession = BallPossession::THEIRS;
    } else {
        newPossession = BallPossession::LOOSE;
    }

    // 状态变化时记录时间
    if (newPossession != data->ballPossession) {
        data->ballPossession = newPossession;
        data->ballPossessionLastUpdate = now;
    }
}
```

---

## 13. 增加定位球战术站位

### 建议说明

在角球、任意球等定位球时，机器人自动移动到预定战术位置。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - 新增定位球站位函数

```cpp
void Brain::setupCornerKick() {
    int playerId = config->playerId;
    auto fd = config->fieldDimensions;

    Point targetPos;

    // 根据 playerId 分配位置
    if (playerId == 1) {
        // 主攻手：近门柱
        targetPos.x = fd.length / 2.0 - fd.goalAreaWidth;
        targetPos.y = fd.goalWidth / 4.0;
    } else if (playerId == 2) {
        // 辅助前锋：远门柱
        targetPos.x = fd.length / 2.0 - fd.goalAreaWidth;
        targetPos.y = -fd.goalWidth / 4.0;
    } else if (playerId == 3) {
        // 中场：点球点附近
        targetPos.x = fd.length / 2.0 - fd.penaltyAreaLength / 2.0;
        targetPos.y = 0.0;
    } else if (playerId == 4) {
        // 后卫：禁区外
        targetPos.x = fd.length / 2.0 - fd.penaltyAreaLength - 1.0;
        targetPos.y = fd.penaltyAreaWidth / 4.0;
    } else {
        // 守门员：留在本方禁区
        targetPos.x = -fd.length / 2.0 + fd.goalAreaWidth;
        targetPos.y = 0.0;
    }

    // 移动到目标位置
    // ...
}
```

---

## 14. 增加拖延策略

### 建议说明

在比赛剩余时间少且领先时，采用拖延战术，控制球在角落消耗时间。

### 代码实现

**修改位置**: `src/brain/src/brain_tree.cpp` - 新增拖延策略函数

```cpp
void Brain::executeTimeWasting() {
    auto fd = config->fieldDimensions;
    Point targetPos;

    // 选择角落位置
    if (data->robotPoseToField.y > 0) {
        targetPos.x = fd.length / 2.0 - fd.goalAreaLength;
        targetPos.y = fd.width / 2.0 - 1.0;
    } else {
        targetPos.x = fd.length / 2.0 - fd.goalAreaLength;
        targetPos.y = -fd.width / 2.0 + 1.0;
    }

    // 如果已到达角落，保持控球
    double dist = norm(targetPos.x - data->robotPoseToField.x,
                       targetPos.y - data->robotPoseToField.y);

    if (dist < 0.5) {
        // 小幅度移动，保持球在控制范围内
        brain->client->setVelocity(0.1, 0, 0);
    } else {
        // 移动到角落
        // ...
    }
}
```

在 `StrikerDecide` 中调用：

```cpp
if (brain->data->enableTimeWasting && brain->data->ballPossession == BallPossession::OURS) {
    executeTimeWasting();
    return NodeStatus::SUCCESS;
}
```

---

## 15. 开球时传中的修改样例

### 15.1 修改 brain_data.h

**文件**: `src/brain/include/brain_data.h`

```cpp
    // 运动规划
    double kickDir = 0.; // 在决策中规划的踢球方向, field 坐标系
    string kickType = "shoot"; // "shoot" | "cross" | "block"
    bool isDirectShoot = false; // 在直接任意球开球的时候, 这个值会为 true; 执行了踢球动作或超过规定时间, 这个值会被 handleSpecialStates 重置为 false
    bool useFakeGoal = false; // 是否使用假球门 (用于 freekick 传中场景, 让 visual kick 往中场传球而不是往球门踢)
    Point2D fakeGoalPos = {0.0, 0.0}; // 假球门在球场坐标系中的位置
    // [原内容] bool isDirectShoot = false; 之后直接是空行和 "// 双机配合" 注释
    // [新增] 上面两行 useFakeGoal 和 fakeGoalPos 是新增的
```

### 15.2 修改 brain_tree.cpp

**文件**: `src/brain/src/brain_tree.cpp`

#### 修改点 A — CalcKickDir::tick() 第一个 cross 分支 (~L900)

```cpp
    if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        brain->data->useFakeGoal = false;  // [新增]
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
```

#### 修改点 B — CalcKickDir::tick() 第二个 freekick cross 分支 (~L907)

```cpp
    else if (
        brain->data->isFreekickKickingOff
        && brain->isPrimaryStriker()
        && !brain->data->isDirectShoot
        && (bPos.x < fd.length/2.0 - fd.goalAreaWidth && bPos.x > -fd.length/2.0 + fd.penaltyAreaWidth)
    ) {
        // 在开任意球时, 决策要不要传中
        brain->data->kickType = "cross";
        brain->data->useFakeGoal = false;  // [新增]
        color = 0xFF00FFFF;
        auto ballPos = brain->data->ball.posToField;
        auto fd = brain->config->fieldDimensions;
        if (ballPos.y > fd.width / 2.0  * 0.8) brain->data->kickDir = - M_PI / 2.0;
        if (ballPos.y < -fd.width / 2.0  * 0.8) brain->data->kickDir =  M_PI / 2.0;
    }
```

#### 修改点 C — CalcKickDir::tick() 新增 freekick 传中假球门分支 (~L920)

```cpp
    // [原内容] 此分支不存在，原代码从第二个 else-if 直接跳到 else if (brain->isDefensing())
    else if (
        brain->data->isFreekickKickingOff
        && brain->isPrimaryStriker()
        && !brain->data->isDirectShoot
        && (brain->data->realGameSubState == "CORNER_KICK"
            || brain->data->realGameSubState == "GOAL_KICK"
            || brain->data->realGameSubState == "THROW_IN"
            || brain->data->realGameSubState == "INDIRECT_FREEKICK")
    ) {
        // 边线球 / 角球 / 门球 / 间接任意球: 使用假球门传中到中场
        // 假球门位置: 球场中场, 偏向球所在的一侧
        double fakeGoalX = 0.0;
        double fakeGoalY = 0.0;
        if (fabs(bPos.y) > fd.width / 2.0 * 0.5) {
            fakeGoalY = (bPos.y > 0 ? -1.0 : 1.0);
        }
        brain->data->useFakeGoal = true;
        brain->data->fakeGoalPos = {fakeGoalX, fakeGoalY};
        brain->data->kickType = "shoot";
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            fakeGoalY - bPos.y,
            fakeGoalX - bPos.x
        );
    }
```

#### 修改点 D — CalcKickDir::tick() defensing 分支 (~L949)

```cpp
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        brain->data->useFakeGoal = false;  // [新增]
        color = 0xFFFF00FF;
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );
    }
```

#### 修改点 E — CalcKickDir::tick() default shoot 分支 (~L958)

```cpp
    } else { // default to shoot
        brain->data->kickType = "shoot";
        brain->data->useFakeGoal = false;  // [新增]
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
    }
```

#### 修改点 F — GoToFreekickPosition::onRunning() 新增 isFreekickPassScenario 判断和 rank==1 站位 (~L520)

```cpp
    const int rank = brain->data->tmMyCostRank;
    bool isFreekickPassScenario = brain->data->isFreekickKickingOff
        && !brain->data->isDirectShoot
        && (brain->data->realGameSubState == "CORNER_KICK"
            || brain->data->realGameSubState == "GOAL_KICK"
            || brain->data->realGameSubState == "THROW_IN"
            || brain->data->realGameSubState == "INDIRECT_FREEKICK");

    if (side == "attack") {
        double attackDist = 0.7;
        getInput("attack_dist", attackDist);

        if (rank == 0) {
            targetPose.x = ballPos.x - attackDist * cos(kickDir);
            targetPose.y = ballPos.y - attackDist * sin(kickDir);
            targetPose.theta = kickDir;
        } else if (rank == 1) {
            if (isFreekickPassScenario) {
                // 任意球传中场景: rank==1 的机器人站在中场假球门位置, 准备接球后射门
                double receiveX = brain->data->fakeGoalPos.x;
                double receiveY = brain->data->fakeGoalPos.y;
                targetPose.x = receiveX - 1.0;
                targetPose.y = receiveY;
                targetPose.theta = atan2(ballPos.y - receiveY, ballPos.x - receiveX);
            } else {
                targetPose.x = ballPos.x - 2.0 * cos(defenseDir);
                targetPose.y = ballPos.y - 2.0 * sin(defenseDir);
                targetPose.theta = defenseDir;
            }
        } else if (rank == 2) {
            // ...
        }
    }
```

#### 修改点 G — StrikerDecide::tick() 新增 useFakeGoal 分支 (~L1073)

```cpp
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } else if (
        brain->data->useFakeGoal
        && brain->data->tmImLead
        && brain->data->tmMyCostRank == 0
        && ballRange < autoVisualKickEnableDistMax
        && ballRange > autoVisualKickEnableDistMin
        && fabs(ballYaw) < autoVisualKickEnableAngle * 1.5
    ) {
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
        color = 0xFF00FFFF;
    } else if (
        enableAutoVisualKick &&
        // ...
    )
```

#### 修改点 H — StrikerDecide::tick() kick 分支中重置 useFakeGoal (~L1131)

```cpp
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; // 只要进一次 kick, 就不算是 kickoff 阶段了.
        brain->data->useFakeGoal = false;  // [新增]
    }
```

### 15.3 修改 brain.cpp

**文件**: `src/brain/src/brain.cpp`

#### 修改点 I — pubKickMsg() 中使用假球门坐标 (~L349)

```cpp
    double goal_x = config->fieldDimensions.length / 2;
    double goal_y = 0.0;

    if (data->useFakeGoal) {
        goal_x = data->fakeGoalPos.x;
        goal_y = data->fakeGoalPos.y;
    }

    Pose2D goalPose;
```

#### 修改点 J — pubKickMsg() 中假球门功率分支 (~L373)

```cpp
    if (data->useFakeGoal) {
        power = config->RLVisionKickLowPassPower;
        reason_power05 = 4;
    } else if (ball_in_opponent_corner) {
        // ...
    }
```

#### 修改点 K — handleSpecialStates() 超时重置 (~L433)

```cpp
    } else if (msecsSince(data->freekickKickoffStartTime) > KICKOFF_DURATION * 1000) {
        data->isFreekickKickingOff = false;
        data->isDirectShoot = false;
        data->useFakeGoal = false;  // [新增]
    }
```
