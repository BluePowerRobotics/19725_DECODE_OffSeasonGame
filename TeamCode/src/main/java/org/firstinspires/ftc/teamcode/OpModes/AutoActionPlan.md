# AutoAction 自动模式逻辑说明

## 一、整体架构

`AutoAction` 是 FTC 机器人的自动模式 OpMode，实现了完整的自动游戏流程：搜索球 → 收集球 → 移动到射击区 → 射击。

## 二、初始化阶段

### 2.1 队伍颜色选择

```java
while (opModeInInit() || !initStarted) {
    if (gamepad1.a) teamColor = TEAM_COLOR.BLUE;
    if (gamepad1.b) teamColor = TEAM_COLOR.RED;
    
    switch (teamColor) {
        case BLUE: targetTagId = 20; break;
        case RED: targetTagId = 24; break;
    }
}
```

| 按键 | 功能 | 目标 AprilTag ID |
|------|------|----------------|
| A | 选择蓝队 | 20 |
| B | 选择红队 | 24 |

### 2.2 组件初始化

```java
ActionRunner actionRunner = new ActionRunner();
chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
turret = new Turret(hardwareMap, telemetry);
sweeper = new Sweeper(hardwareMap, telemetry);
tracker = new Tracker(hardwareMap);
tracker.start();

// 初始化吃球位姿访问状态
Pose2d[] eatPoses = (teamColor == TEAM_COLOR.RED) ? HypParams.EatPosesRed : HypParams.EatPosesBlue;
eatPoseReached = new boolean[eatPoses.length];
currentEatPoseIndex = 0;
```

| 组件 | 作用 |
|------|------|
| ActionRunner | 管理动作队列，串行执行动作 |
| Chassis | 底盘控制，负责移动 |
| Turret | 炮塔控制，负责瞄准和发射 |
| Sweeper | 清扫器，收集球 |
| Tracker | 视觉追踪器，检测目标球 |
| eatPoseReached | 记录每个吃球位姿是否已到达 |

### 2.3 吃球位姿配置

在 `HypParams` 中定义了红蓝两队的吃球位姿列表：

```java
// 红队吃球位姿列表
public static Pose2d[] EatPosesRed = {
    new Pose2d(0, 0, 0),
    new Pose2d(10, -20, Math.toRadians(45)),
    new Pose2d(20, -40, Math.toRadians(90))
};

// 蓝队吃球位姿列表
public static Pose2d[] EatPosesBlue = {
    new Pose2d(0, 0, 0),
    new Pose2d(10, 20, Math.toRadians(-45)),
    new Pose2d(20, 40, Math.toRadians(-90))
};
```

## 三、主循环逻辑

### 3.1 状态机流程图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        AutoAction 主循环                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                    while (opModeIsActive())                         │    │
│  │  ┌─────────────────────────────────────────────────────────────┐    │    │
│  │  │ RobotPosition.getInstance().update()                        │    │    │
│  │  │ actionRunner.update()                                       │    │    │
│  │  └─────────────────────────────────────────────────────────────┘    │    │
│  │                           ↓                                         │    │
│  │                if (!actionRunner.isBusy())                           │    │
│  │                           ↓                                         │    │
│  │    ┌─────────────────┬─────────────────┬─────────────────────────┐  │    │
│  │    ↓                 ↓                 ↓                         │  │    │
│  │  [满仓 &&         [非空 &&          [空仓 ||                     │  │    │
│  │   不在射击区]      在射击区]         (不满仓 &&                   │  │    │
│  │                          │           不在射击区)]                 │  │    │
│  │                          ↓                 ↓                       │  │    │
│  │     GoToShootingAreaAction    ┌─────────────────────────────┐    │  │    │
│  │     ShootAction               ↓                           ↓    │  │    │
│  │                    [有未到达的吃球位姿]            [所有位姿已到达]    │  │    │
│  │                           ↓                           ↓    │  │    │
│  │              ┌─────────────┴─────────────┐   ┌──────┴──────┐    │  │    │
│  │              ↓                           ↓   ↓             ↓    │  │    │
│  │      [刚到达吃球位姿]            [前往下一个]   [检测到目标]    [未检测到]    │  │    │
│  │              ↓                           ↓   ↓             ↓    │  │    │
│  │     ┌────────┴────────┐         GoToEatPose  EatAction  SearchAction│  │    │
│  │     ↓                 ↓                                            │  │    │
│  │  [视野内有球]    [视野内无球]                                        │  │    │
│  │     ↓                 ↓                                            │  │    │
│  │  EatAction     标记已到达，前往下一个                                 │  │    │
│  │                                                                    │  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.2 状态转换条件

| 条件 | 动作 | 说明 |
|------|------|------|
| `isFull() && !isAbleToShoot()` | GoToShootingAreaAction | 球仓已满但不在射击区，需要移动到射击区 |
| `!isEmpty() && isAbleToShoot()` | ShootAction | 有球且在射击区，执行射击 |
| `isEmpty() \|\| (!isFull() && !isAbleToShoot())` | GoToEatPose / EatAction / SearchAction | 需要收集球 |

### 3.3 吃球位姿访问逻辑

```java
// 检查是否有未到达的吃球位姿
boolean hasUnreachedPose = false;
for (int i = 0; i < eatPoseReached.length; i++) {
    if (!eatPoseReached[i]) {
        hasUnreachedPose = true;
        break;
    }
}

if (hasUnreachedPose) {
    if (lastActionType.equals("GoToEatPose")) {
        // 已到达吃球位姿，检查视野内是否有球
        if (tracker.getHasTarget()) {
            actionRunner.add(new EatAction(chassis, tracker, sweeper));
            lastActionType = "Eat";
        } else {
            // 没有球，标记当前位姿已到达，前往下一个
            eatPoseReached[currentEatPoseIndex] = true;
            for (int i = 0; i < eatPoseReached.length; i++) {
                if (!eatPoseReached[i]) {
                    currentEatPoseIndex = i;
                    break;
                }
            }
            actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
            lastActionType = "GoToEatPose";
        }
    } else {
        // 前往第一个未到达的吃球位姿
        for (int i = 0; i < eatPoseReached.length; i++) {
            if (!eatPoseReached[i]) {
                currentEatPoseIndex = i;
                break;
            }
        }
        actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
        lastActionType = "GoToEatPose";
    }
} else {
    // 所有吃球位姿都已到达，执行原来的逻辑
    if(tracker.getHasTarget()){
        actionRunner.add(new EatAction(chassis, tracker, sweeper));
        lastActionType = "Eat";
    } else {
        actionRunner.add(new SearchAction(chassis, tracker, sweeper, teamColor));
        lastActionType = "Search";
    }
}
```

## 四、动作详解

### 4.1 GoToShootingAreaAction

将机器人移动到射击区域。

### 4.2 ShootAction

执行射击动作：
1. 使用 AprilTag 瞄准目标
2. 计算弹道参数
3. 发射球

### 4.3 EatAction

收集球动作：
1. 跟踪目标球
2. 移动到球的位置
3. 启动清扫器收集

### 4.4 SearchAction

搜索动作（关键逻辑）：

```java
public boolean run(TelemetryPacket packet) {
    tracker.update();

    // 如果检测到目标，提前中止轨迹
    if (tracker.getBestTarget() != null) {
        sweeper.setStop();
        return false;  // 提前中止，让 ActionRunner 调度 EatAction
    }

    // 根据队伍颜色选择搜索轨迹
    if (teamColor == Chassis.TEAM_COLOR.BLUE) {
        // 蓝队：下方搜索轨迹
        trajectoryAction = drive.actionBuilder(currentPose)
            .splineTo(new Vector2d(-15, -48), -Math.PI / 2)
            .lineTo(new Vector2d(-57, -48))
            .lineTo(new Vector2d(-15, -48))
            .build();
    } else {
        // 红队：上方搜索轨迹
        trajectoryAction = drive.actionBuilder(currentPose)
            .splineTo(new Vector2d(-15, 48), -Math.PI / 2)
            .lineTo(new Vector2d(-57, 48))
            .lineTo(new Vector2d(-15, 48))
            .build();
    }

    // 执行搜索轨迹
    sweeper.setEat();
    // ... 沿预定轨迹移动
}
```

**搜索轨迹（按队伍颜色）**：

| 队伍颜色 | Y坐标 | 搜索路径 |
|----------|-------|----------|
| **蓝队 (BLUE)** | -48 | (-15,-48) → (-57,-48) → (-15,-48) |
| **红队 (RED)** | +48 | (-15,48) → (-57,48) → (-15,48) |

**提前中止机制**：在搜索过程中，如果 `tracker` 检测到目标球，立即停止轨迹，让 `ActionRunner` 切换到 `EatAction`。

### 4.5 GoToEatPose（新增）

移动到预设的吃球位姿：

```java
public class GoToEatPose implements Action {
    private final Chassis chassis;
    private final Sweeper sweeper;
    private final Pose2d targetPose;
    private Action trajectoryAction;
    private boolean trajectoryStarted;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();
        sweeper.setStop();
        sweeper.update();

        if (!trajectoryStarted) {
            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
            trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                    .lineToSplineHeading(targetPose)
                    .build();
            trajectoryStarted = true;
        }

        if (trajectoryAction != null) {
            boolean running = trajectoryAction.run(packet);
            if (!running) {
                trajectoryStarted = false;
                trajectoryAction = null;
                return false;  // 到达目标位姿
            }
        }
        return true;
    }
}
```

## 五、关键设计特点

### 5.1 动作队列机制

使用 `ActionRunner` 管理动作队列，实现：
- 动作串行执行
- 非阻塞更新
- 动作优先级管理

### 5.2 状态驱动

基于机器人状态（球仓状态、位置状态）决定下一步动作，实现智能决策。

### 5.3 吃球位姿管理

通过预设的吃球位姿列表和访问状态标记，实现有序的吃球路径规划：

| 队伍颜色 | 吃球位姿列表 |
|----------|-------------|
| **红队** | `EatPosesRed` |
| **蓝队** | `EatPosesBlue` |

### 5.4 视觉反馈

`Tracker` 实时更新目标检测状态，支持：
- 目标颜色识别（紫色/绿色）
- 低通滤波平滑
- 距离得分排序

### 5.5 动态切换

`SearchAction` 和 `GoToEatPose` 支持中途切换到 `EatAction`，提高响应速度。

## 六、代码优化建议

### 6.1 状态机重构

当前使用多个 if 语句判断状态，可以考虑引入状态机模式：

```java
enum RobotState {
    SEARCHING, EATING, GOING_TO_EAT_POSE, GOING_TO_SHOOTING_AREA, SHOOTING
}
```

### 6.2 动作优先级

当前多个条件可能同时满足（如满仓且在射击区），建议使用 if-else 结构确保唯一动作选择。

### 6.3 日志增强

增加关键状态的日志输出，便于调试和问题定位。

### 6.4 吃球位姿动态配置

考虑将吃球位姿配置为可动态调整的参数，便于现场调试。