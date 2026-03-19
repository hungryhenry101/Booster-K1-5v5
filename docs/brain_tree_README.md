# brain_tree.cpp Action Nodes 说明文档

本文档详细说明了 `brain_tree.cpp` 中定义的所有 Action Nodes 的功能、参数和实现逻辑。

---

## 目录

1. [球追踪与找球相关](#球追踪与找球相关)
2. [踢球相关](#踢球相关)
3. [决策相关](#决策相关)
4. [定位相关](#定位相关)
5. [移动相关](#移动相关)
6. [守门员相关](#守门员相关)
7. [角色与策略相关](#角色与策略相关)
8. [身体控制相关](#身体控制相关)
9. [调试相关](#调试相关)

---

## 球追踪与找球相关

### RobotFindBall
**功能**: 机器人通过身体旋转来寻找球

**输入参数**:
- `vyaw_limit` (double): 旋转速度限制

**实现逻辑**:
- 检测到球时停止并返回 SUCCESS
- 未检测到球时,向记忆中球的方向旋转(`vyaw_limit * _turnDir`)
- 如果记忆中的球位置太近(<0.3m),会考虑后退(当前注释掉)

**状态**: AsyncActionNode (onStart/onRunning/onHalted)

---

### Chase
**功能**: 追球节点,控制机器人以智能方式接近球

**输入参数**:
- `vx_limit` (double): X方向速度限制
- `vy_limit` (double): Y方向速度限制
- `vtheta_limit` (double): 角速度限制
- `dist` (double): 目标距离球的距离
- `safe_dist` (double): 绕行时的安全距离

**实现逻辑**:
1. **Circle Back模式**: 当机器人在球后方时,绕到球前方
   - 目标位置: 球后方距离`dist`处
   - 决定从左侧或右侧绕行(通过`_dir`变量防止震荡)
   - 横向移动并调整朝向

2. **Chase模式**: 正常追球
   - 目标位置: 球前方距离`dist`处,沿`kickDir`方向
   - 距离远时主要转向,距离近时同时前进
   - 使用sigmoid函数平滑过渡

3. **安全机制**:
   - 检测到球出界时停止
   - 防止乌龙球(接近球且方向不对时减速)
   - 根据距离和角度调整速度

**状态**: SyncActionNode (tick)

---

### SimpleChase
**功能**: 简化的追球节点

**输入参数**:
- `stop_dist` (double): 停止距离
- `stop_angle` (double): 停止角度(当前注释掉)
- `vx_limit` (double): X方向速度限制
- `vy_limit` (double): Y方向速度限制

**实现逻辑**:
- 直接向球的位置移动
- 速度由球在机器人坐标系中的位置决定
- 使用指数函数在距离远时优先转向
- 到达停止距离后停止

**状态**: SyncActionNode (tick)

---

### CamTrackBall
**功能**: 控制相机头部持续跟踪球

**输入参数**: 无

**实现逻辑**:
1. **检测到球**:
   - 计算球在图像中的位置偏移量
   - 如果偏移量在容差范围内,返回SUCCESS
   - 否则,移动头部使球居中(使用平滑系数3.5)

2. **未检测到球**:
   - 根据记忆中的球位置或队友球位置移动头部

3. **参数**:
   - `pixToleranceX/Y`: 像素容差(相机尺寸的30%)
   - `smoother`: 头部运动平滑系数

**状态**: SyncActionNode (tick)

---

### CamFindBall
**功能**: 通过预设的头部运动序列找球

**输入参数**: 无

**实现逻辑**:
- 执行6步头部扫描序列:
  1. 低俯仰角 + 左偏航
  2. 低俯仰角 + 中间偏航
  3. 低俯仰角 + 右偏航
  4. 高俯仰角 + 右偏航
  5. 高俯仰角 + 中间偏航
  6. 高俯仰角 + 左偏航
- 每步间隔800ms
- 50秒后重新开始
- 检测到球时返回SUCCESS

**状态**: SyncActionNode (tick)

---

### CamFastScan
**功能**: 快速扫描相机头部

**输入参数**:
- `msecs_interval` (double): 每步间隔时间(ms)

**实现逻辑**:
- 快速执行6步头部扫描序列
- 通过`onStart`/`onRunning`状态机实现
- 最多执行6步后返回SUCCESS

**状态**: AsyncActionNode (onStart/onRunning)

---

### CamScanField
**功能**: 周期性扫描场地

**输入参数**:
- `low_pitch` (double): 低俯仰角
- `high_pitch` (double): 高俯仰角
- `left_yaw` (double): 左偏航角
- `right_yaw` (double): 右偏航角
- `msec_cycle` (double): 扫描周期时间(ms)

**实现逻辑**:
- 基于系统时间的周期性扫描
- 前半周期使用高俯仰角,后半周期使用低俯仰角
- 偏航角在左右边界之间三角波运动

**状态**: SyncActionNode (tick)

---

## 踢球相关

### Adjust
**功能**: 调整位置以便更好地踢球

**输入参数**:
- `turn_threshold` (double): 转向阈值
- `vx_limit` (double): X方向速度限制
- `vy_limit` (double): Y方向速度限制
- `vtheta_limit` (double): 角速度限制
- `range` (double): 目标距离球的距离
- `position` (string): 位置类型("defense"或"attack")

**实现逻辑**:
1. **角度差过大(>60度)**: 仅横向移动,防止碰到球
2. **角度差不大**: 向目标位置移动(球后方`range`距离,沿`kickDir`方向)
3. `position="defense"`时,`kickDir`指向球门方向
4. 如果球朝向角度超过阈值,则调整朝向

**状态**: SyncActionNode (tick)

---

### Kick
**功能**: 执行踢球动作

**输入参数**:
- `vx_limit` (double): X方向速度限制
- `vy_limit` (double): Y方向速度限制
- `min_msec_kick` (int): 最小踢球时间(ms)
- `msecs_stablize` (double): 稳定时间(ms)
- `speed_limit` (double): 踢球速度限制

**实现逻辑**:
1. **onStart**:
   - 检测障碍物,必要时后退
   - 决定进入"stablize"或"kick"状态
   - 低风险时先稳定再踢球

2. **onRunning - Stabilize状态**:
   - 后退稳定`msecs_stablize`毫秒
   - 然后切换到kick状态

3. **onRunning - Kick状态**:
   - 使用crabWalk向球方向移动
   - 根据球距离估算所需时间
   - 如果球被踢远或丢失,中止踢球
   - 逐渐加速(`_speed += 0.08`)
   - 软开球时使用较低速度

4. **onHalted**: 重置时间戳

**状态**: AsyncActionNode (onStart/onRunning/onHalted)

---

### CalcKickDir
**功能**: 计算最佳踢球方向

**输入参数**:
- `cross_threshold` (double): 传中角度阈值

**实现逻辑**:
1. **传中(Cross)**: 两个门柱角度差小于阈值时
   - 目标: 球门线中点附近
2. **开球时传中**: 特殊逻辑在边线附近传中
3. **防守(Block)**: 防守时踢向球门反方向
4. **射门(Shoot)**: 默认情况,踢向球门

输出: 设置`brain->data->kickDir`和`brain->data->kickType`

**状态**: SyncActionNode (tick)

---

## 决策相关

### StrikerDecide
**功能**: 前锋决策节点,决定当前行为模式

**输入参数**:
- `chase_threshold` (double): 追球距离阈值
- `decision_in` (string): 上次决策
- `position` (string): 位置

**输出**:
- `decision_out` (string): 新决策(find/chase/adjust/kick/cross/safe_shoot/assist)

**实现逻辑**:
1. **find**: 不知道球位置或不可靠时
2. **assist**: 队友控球时
3. **chase**: 球距离大于阈值
4. **kick/cross**: 角度合适且距离足够
   - `kickType="cross"`时决策为`cross`
   - 低威胁时为`safe_shoot`
   - 否则为`kick`
5. **adjust**: 其他情况

**判断条件**:
- `angleGoodForKick/Shoot`: 角度是否适合踢球
- `shootPossible`: 是否能射门(距离、角度、位置都合适)
- `reachedKickDir`: 是否到达踢球方向
- `avoidKick`: 是否需要避免碰撞

**状态**: SyncActionNode (tick)

---

### GoalieDecide
**功能**: 守门员决策节点

**输入参数**:
- `chase_threshold` (double): 追球距离阈值
- `decision_in` (string): 上次决策

**输出**:
- `decision_out` (string): 新决策(find/retreat/chase/kick/adjust)

**实现逻辑**:
1. **find**: 不知道球位置
2. **retreat**: 球在前方时后退
3. **chase**: 球距离大于阈值
4. **kick**: 角度合适(机器人-球-门柱角度在[-90°,90°])
5. **adjust**: 其他情况

**状态**: SyncActionNode (tick)

---

### DecideCheckBehind
**功能**: 决定是否需要检查身后

**输入参数**: 无

**实现逻辑**:
- 计算球场地角相对于机器人的角度范围
- 如果所有角落都在机器人的正前方([-103°,103°]),则不需要检查身后
- 否则设置`need_check_behind=true`

**状态**: SyncActionNode (tick)

---

## 定位相关

### SelfLocate
**功能**: 通用自定位节点,使用粒子滤波

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `mode` (string): 定位模式("enter_field"/"face_forward"/"trust_direction"/"fall_recovery")

**实现逻辑**:
1. 检查是否需要重新定位(基于时间间隔)
2. 根据`mode`设置约束条件:
   - `enter_field`: 在入口位置(x,y,theta的合理范围)
   - `face_forward`: 面向前方
   - `trust_direction`: 信任当前方向,允许一定漂移
   - `fall_recovery`: 倒地恢复后定位
3. 调用粒子滤波定位器
4. 验证结果(残差检查)
5. 校准里程计

**状态**: SyncActionNode (tick)

---

### SelfLocateEnterField
**功能**: 入场时定位,自动判断左右侧

**输入参数**:
- `msecs_interval` (double): 重新定位间隔

**实现逻辑**:
1. 分别尝试左右入口位置的定位
2. 比较结果,选择成功或残差更小的
3. 通过语音播报进入哪一侧
4. 成功时校准里程计

**状态**: SyncActionNode (tick)

---

### SelfLocateLocal
**功能**: 使用局部特征的快速定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔

**实现逻辑**:
1. **SinglePenalty**: 单个罚球点定位
   - 判断是己方还是对方罚球点
   - 计算假设位姿
   - 用其他标记验证

2. **DoubleX**: 两个X标记定位
   - 检查Y方向距离应为2倍圆半径
   - 计算球场中心偏移
   - 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocate1P
**功能**: 基于单个罚球点的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到单个罚球点
2. 判断是己方还是对方罚球点
3. 计算偏移量并生成假设位姿
4. 用其他标记验证残差
5. 静态时允许更大的距离和漂移

**状态**: SyncActionNode (tick)

---

### SelfLocate1M
**功能**: 基于任意单个标记的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到最近的标记
2. 排除容易误识别的标记(LOLG/LORG/LSLG/LSRG)
3. 检查距离和视野中心位置
4. 计算偏移量并验证

**状态**: SyncActionNode (tick)

---

### SelfLocate2X
**功能**: 基于两个X标记的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到两个X标记
2. 检查方向(Y方向)和距离(应为2倍圆半径)
3. 计算中心点偏移
4. 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocate2T
**功能**: 基于两个T标记的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到一对X方向距离小、Y方向距离符合T标记模式的标记
2. 计算中点
3. 匹配到四个可能的场地T标记位置
4. 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocateLT
**功能**: 基于一个L标记和一个T标记的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到X方向距离为goalAreaLength的L-T标记对
2. 计算中点
3. 匹配到四个可能的场地L-T位置
4. 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocatePT
**功能**: 基于一个门柱和一个T标记的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到X方向和Y方向距离符合P-T模式的标记对
2. 使用T标记位置
3. 匹配到四个可能的场地P-T位置
4. 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocateBorder
**功能**: 基于边线的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到最佳的TouchLine、GoalLine和MiddleLine
2. 计算到这些线的垂直距离
3. 根据线类型计算X或Y的校准量
4. 可以同时使用两条正交的线
5. 验证并校准

**状态**: SyncActionNode (tick)

---

### SelfLocateLine
**功能**: 基于任意场地线的定位

**输入参数**:
- `msecs_interval` (double): 重新定位间隔
- `max_dist` (double): 最大允许距离
- `max_drift` (double): 最大允许漂移
- `validate` (bool): 是否验证

**实现逻辑**:
1. 找到所有符合条件的场地线
2. 按距离排序
3. 与标准场地线匹配(考虑距离和角度)
4. 选择匹配度最好的一条或两条(正交)线
5. 根据线的方向计算校准量
6. 验证并校准

**辅助方法**:
- `lineToLineAvgDist`: 计算两条线的平均距离

**状态**: SyncActionNode (tick)

---

## 移动相关

### SetVelocity
**功能**: 直接设置机器人速度

**输入参数**:
- `x` (double): X方向速度
- `y` (double): Y方向速度
- `theta` (double): 角速度

**实现逻辑**:
- 直接调用`brain->client->setVelocity(x, y, theta)`

**状态**: SyncActionNode (tick)

---

### StepOnSpot
**功能**: 原地踏步

**输入参数**: 无

**实现逻辑**:
- 生成一个小的随机X方向速度(±0.01 m/s)
- 调用`setVelocity`实现踏步效果

**状态**: SyncActionNode (tick)

---

### TurnOnSpot
**功能**: 原地旋转指定角度

**输入参数**:
- `rad` (double): 旋转角度(弧度)
- `towards_ball` (bool): 是否朝向球旋转

**实现逻辑**:
1. **onStart**:
   - 记录起始角度
   - 累计角度初始化为0
   - 如果`towards_ball=true`,根据球在图像中的位置决定旋转方向

2. **onRunning**:
   - 累计旋转角度
   - 当达到目标角度或超时(5秒)时停止
   - 使用比例控制调整旋转速度

**状态**: AsyncActionNode (onStart/onRunning)

---

### MoveToPoseOnField
**功能**: 移动到场地上指定位置

**输入参数**:
- `x`, `y`, `theta` (double): 目标位姿
- `long_range_threshold` (double): 长距离阈值
- `turn_threshold` (double): 转向阈值
- `vx_limit`, `vy_limit`, `vtheta_limit` (double): 速度限制
- `x_tolerance`, `y_tolerance`, `theta_tolerance` (double): 容差
- `avoid_obstacle` (bool): 是否避障

**实现逻辑**:
- 直接调用`brain->client->moveToPoseOnField2`
- 该方法实现了完整的位姿控制(包括长距离转向、接近、对齐)

**状态**: SyncActionNode (tick)

---

### GoToReadyPosition
**功能**: 移动到准备位置

**输入参数**:
- `dist_tolerance` (double): 距离容差
- `theta_tolerance` (double): 角度容差
- `vx_limit`, `vy_limit` (double): 速度限制

**实现逻辑**:
根据角色和排名设置目标位置:

**前锋**:
- Rank 0: 中圈附近(x=-circleRadius/-circleRadius*2)
- Rank 1: 中圈附近偏左
- Rank 2: 罚球区左上
- Rank 3: 罚球区左下

**守门员**:
- 球门线内(x=-length/2+goalAreaLength, y=0)

开球时Rank 0/1位置更靠近中圈。

**状态**: SyncActionNode (tick)

---

### GoToGoalBlockingPosition
**功能**: 移动到球门阻挡位置

**输入参数**:
- `dist_tolerance` (double): 距离容差
- `theta_tolerance` (double): 角度容差
- `dist_to_goalline` (double): 距离球门线的距离
- `vx_limit`, `vy_limit` (double): 速度限制

**实现逻辑**:
根据角色计算目标位置:

**前锋**: 限制在球后方1.5m或距离球门线`dist_to_goalline`

**守门员**:
- Y坐标: 球的Y坐标按比例缩放到球门线或罚球区宽度
- 范围: 前锋为球门宽度,守门员为罚球区宽度
- X坐标: 距离球门线`dist_to_goalline`

朝向球的方向。

**状态**: SyncActionNode (tick)

---

### GoToFreekickPosition
**功能**: 移动到任意球位置

**输入参数**:
- `side` (string): "attack"或"defense"
- `attack_dist` (double): 进攻距离
- `vx_limit`, `vy_limit` (double): 速度限制
- `dist_tolerance`, `theta_tolerance` (double): 容差

**实现逻辑**:

**Attack模式**:
- Rank 0: 球后方`attack_dist`距离,沿`kickDir`方向
- Rank 1: 球后方2m,沿防守方向
- Rank 2/3: 罚球区角落

**Defense模式**:
- Rank 0: 球后方2m,沿防守方向
- Rank 1: 球后方1.5m,沿防守方向
- Rank 2/3: 罚球区角落

使用`moveToPoseOnField3`实现,接近时关闭避障。

**状态**: AsyncActionNode (onStart/onRunning/onHalted)

---

### GoBackInField
**功能**: 出界后返回场内

**输入参数**:
- `valve` (double): 判断出界的阈值

**实现逻辑**:
- 检查是否出界(x或y超出场地范围±`valve`)
- 如果出界,朝向场地中心移动(速度0.4 m/s)
- 否则停止

**状态**: SyncActionNode (tick)

---

### Assist
**功能**: 辅助节点,移动到合适的防守位置

**输入参数**:
- `dist_tolerance` (double): 距离容差
- `theta_tolerance` (double): 角度容差
- `dist_to_goalline` (double): 距离球门线的距离
- `vx_limit`, `vy_limit` (double): 速度限制

**实现逻辑**:
根据排名计算目标位置:

- Rank 1: 球后方2m,但不超过距离球门线`dist_to_goalline`
- Rank 2: 距离球门线`penaltyDist`,但不超过球后方1m
- Rank 3: 距离球门线`goalAreaLength`,但不超过球后方0.5m

所有位置的Y坐标按比例缩放,可以挡住球的位置。

可选障碍物避免。

**状态**: SyncActionNode (tick)

---

## 守门员相关

### Intercept
**功能**: 守门员拦截球

**输入参数**:
- `squat_dist` (double): 下蹲能挡住的距离
- `stand_block_dist` (double): 站立能挡住的距离

**实现逻辑**:
1. **Stand状态**:
   - 移动到拦截点
   - 如果距离小于`squat_dist`,切换到Squat状态
   - 支持移动阻挡(`use_move`)

2. **Squat状态**:
   - 根据Y坐标决定阻挡方向(left/right/center)
   - 执行下蹲阻挡动作
   - 持续`squatMsecs`后起身
   - 切换`goalie_mode`为"attack"

3. **onHalted**: 起身并说"halt"

**状态**: AsyncActionNode (onStart/onRunning/onHalted)

---

### GoToGoalBlockingPosition
**功能**: 移动到球门阻挡位置(已在移动相关章节说明)

---

## 角色与策略相关

### RoleSwitchIfNeeded
**功能**: 根据场上情况动态切换角色

**输入参数**: 无

**实现逻辑**:
1. 统计场上存活球员数量
2. 如果不满员且自己未受罚,切换为"striker"
3. 如果缺少1名球员,切换为"goal_keeper"
4. 在INITIAL状态,恢复初始角色
5. 角色切换时语音播报

**状态**: SyncActionNode (tick)

---

### DecideCheckBehind
**功能**: 决定是否需要检查身后(已在决策相关章节说明)

---

### Assist
**功能**: 辅助节点(已在移动相关章节说明)

---

## 身体控制相关

### StandStill
**功能**: 站立不动指定时间

**输入参数**:
- `msecs` (double): 站立时间(ms)

**实现逻辑**:
1. **onStart**: 设置速度为0,记录开始时间
2. **onRunning**: 持续保持速度为0,直到时间结束
3. **onHalted**: 修改时间戳以快速重置

**状态**: AsyncActionNode (onStart/onRunning/onHalted)

---

### WaveHand
**功能**: 挥手动作

**输入参数**:
- `action` (string): "start"开始挥手,其他停止挥手

**实现逻辑**:
- 调用`brain->client->waveHand(bool)`

**状态**: SyncActionNode (tick)

---

### MoveHead
**功能**: 移动头部到指定位置

**输入参数**:
- `pitch` (double): 俯仰角
- `yaw` (double): 偏航角

**实现逻辑**:
- 调用`brain->client->moveHead(pitch, yaw)`

**状态**: SyncActionNode (tick)

---

### CheckAndStandUp
**功能**: 检查是否跌倒并尝试起身

**输入参数**: 无

**实现逻辑**:
1. 如果处于惩罚状态,重置恢复状态
2. 如果已跌倒且处于阻尼模式,尝试调用`standUp()`
3. 限制最大尝试次数(`retry_max_count`)
4. 起身完成后增加重试计数
5. 如果站立且处于robocup步态,重置恢复状态

**状态**: SyncActionNode (tick)

---

## 调试相关

### CrabWalk
**功能**: 横向行走调试

**输入参数**:
- `angle` (double): 行走角度
- `speed` (double): 行走速度

**实现逻辑**:
- 调用`brain->client->crabWalk(angle, speed)`

**状态**: SyncActionNode (tick)

---

### AutoCalibrateVision
**功能**: 自动校准视觉参数

**输入参数**: 从blackboard读取:
- `calibrate_state`: "pitch"/"yaw"/"z"
- `calibrate_pitch_center/step`: 俯仰角中心和步长
- `calibrate_yaw_center/step`: 偏航角中心和步长
- `calibrate_z_center/step`: Z轴中心和步长

**实现逻辑**:
1. **onStart**:
   - 将机器人放置到已知位置
   - 生成参数扫描序列(每个方向21个点)
   - 开始运行

2. **onRunning**:
   - 依次发送参数组合
   - 等待参数生效(50ms)
   - 计算当前参数的残差(标记点到最近标记点的平均距离)
   - 累计多个样本求平均
   - 所有参数测试完后,找到残差最小的参数作为中心
   - 步长减半,进入下一个状态(pitch→yaw→z→pitch...)
   - 当步长小于阈值时,输出最终参数

**辅助方法**:
- `_calcResidual`: 计算当前视觉参数的残差

**状态**: AsyncActionNode (onStart/onRunning)

---

### CalibrateOdom
**功能**: 手动校准里程计

**输入参数**:
- `x`, `y`, `theta` (double): 校准后的位姿

**实现逻辑**:
- 调用`brain->calibrateOdom(x, y, theta)`

**状态**: SyncActionNode (tick)

---

### PrintMsg
**功能**: 打印消息到控制台

**输入参数**:
- `msg` (string): 要打印的消息

**实现逻辑**:
- 输出格式: `[MSG] <msg>`

**状态**: SyncActionNode (tick)

---

### PlaySound
**功能**: 播放声音

**输入参数**:
- `sound` (string): 声音名称
- `allow_repeat` (bool): 是否允许重复播放

**实现逻辑**:
- 调用`brain->playSound(sound, allowRepeat)`

**状态**: SyncActionNode (tick)

---

### Speak
**功能**: 文字转语音

**输入参数**:
- `text` (string): 要朗读的文本

**实现逻辑**:
- 避免重复朗读相同的文本
- 调用`brain->speak(text, false)`

**状态**: SyncActionNode (tick)

---

## 附录: 状态机类型

### SyncActionNode
- 单次执行,通过`tick()`方法返回`NodeStatus`
- `SUCCESS`: 节点成功完成
- `FAILURE`: 节点失败
- `RUNNING`: 节点仍在运行(对于异步逻辑也适用)

### AsyncActionNode
- 使用状态机实现,生命周期:
  - `onStart()`: 节点开始时调用,返回`RUNNING`
  - `onRunning()`: 每次tick调用,返回`RUNNING`或`SUCCESS`
  - `onHalted()`: 节点被中断时调用(例如父节点失败)

---

## 代码约定

1. **宏定义**: `REGISTER_BUILDER(Name)`用于注册节点到工厂
2. **日志**: 使用`brain->log`进行日志记录(使用rerun)
3. **输入输出**: 使用`getInput<T>(key)`和`setOutput(key, value)`
4. **Blackboard**: 使用`brain->tree->getEntry<T>(key)`和`setEntry(key, value)`
5. **坐标系统**:
   - Robot坐标系: 以机器人为原点
   - Field坐标系: 以球场中心为原点,X向前,Y向左
6. **角度**: 使用弧度,范围[-π, π]
7. **时间**: 使用毫秒(ms)

---

## 总结

`brain_tree.cpp` 实现了一个完整的足球机器人行为树系统,包含约50个Action Nodes,涵盖了:

- **感知**: 找球、跟踪球、定位
- **决策**: 策略选择、角色切换、踢球决策
- **行动**: 移动、踢球、身体控制
- **调试**: 参数校准、消息输出

所有节点都遵循BehaviorTree.CPP的规范,通过宏注册到工厂,在行为树XML文件中使用。

