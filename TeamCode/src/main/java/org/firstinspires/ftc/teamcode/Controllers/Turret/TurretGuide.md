# Turret 类使用指南

## 1. 核心功能

Turret 类是 FTC 机器人炮台控制系统的核心类，封装了炮台的所有控制功能，包括：

- **自动瞄准**：使用 AprilTag 视觉识别实现自动对准目标
- **角度控制**：精确控制炮台的水平旋转角（roll）和仰角（yaw）
- **发射控制**：计算发射参数并控制电机达到目标速度
- **轨迹计算**：使用 RK4 算法计算小球轨迹，确保命中目标

## 2. 硬件配置

### 2.1 必需硬件

- **Webcam**：用于 AprilTag 视觉识别，配置名称为 "Webcam 1"
- **发射电机**：控制发射轮速度，配置名称为 "shooterMotor"
- **炮台旋转电机**：控制水平旋转，配置名称为 "turretMotor"
- **仰角电机**：控制仰角调整，配置名称为 "yawMotor"

### 2.2 硬件初始化

```java
// 在 OpMode 中初始化 Turret 类
Turret turret = new Turret(hardwareMap, telemetry);
```

## 3. 主要方法

### 3.1 rotate_to(double roll, double yaw)

**功能**：旋转炮台到指定角度

- **参数**：
  - `roll`：水平旋转角（度，逆时针为正）
  - `yaw`：仰角（度）
- **返回值**：是否成功旋转到目标角度

### 3.2 get_angle()

**功能**：获取当前炮台角度

- **返回值**：包含 roll 和 yaw 的数组 `[roll, yaw]`

### 3.3 aim()

**功能**：使用 AprilTag 自动瞄准目标

- **返回值**：目标的仰角和旋转角 `[roll, yaw]`
- **工作原理**：
  1. 检测 AprilTag
  2. 计算角度偏移
  3. 旋转到目标角度
  4. 重复直到角度偏移小于阈值
  5. 未检测到目标时，逆时针旋转 90 度继续搜索

### 3.4 set(double k, double b)

**功能**：设置速度转换参数

- **参数**：
  - `k`：比例系数
  - `b`：截距
- **作用**：用于将物理速度（m/s）转换为电机转速（度/秒）

### 3.5 shoot(double roll, double yaw)

**功能**：设置发射参数并开始异步发射序列

- **参数**：
  - `roll`：目标旋转角
  - `yaw`：目标仰角
- **工作原理**（异步等待模式）：
  1. 计算目标相对位置
  2. 使用 BST 求解器计算发射参数
  3. 设置飞轮目标速度和炮塔目标角度
  4. 标记等待飞轮加速状态，不阻塞主循环
  5. 在 update() 中异步检测飞轮速度达标后自动触发发射

### 3.6 update(boolean shouldAim, boolean shouldShoot, int targetTagId)

**功能**：每帧调用的主更新函数，包含异步发射状态机 + 连续跟踪

- **参数**：
  - `shouldAim`：是否启用视觉瞄准
  - `shouldShoot`：是否发射
  - `targetTagId`：目标 AprilTag ID
- **工作原理**（异步等待模式 + 连续跟踪）：
  1. 更新飞轮和炮塔控制器
  2. 检查发射冷却倒计时，冷却结束后复位状态
  3. 若正在等待飞轮加速：
     - 每帧重新瞄准（AprilTag视觉或位置跟踪）
     - 每帧重新解算 BST 弹道，补偿机器人运动
     - 更新炮塔角度跟随目标
  4. 若速度+角度同时达标 → 发射
  5. 若未在等待 → 执行瞄准 → 设置发射参数（进入等待状态）
- **超时保护**：飞轮加速超时 500ms 自动放弃
- **冷却间隔**：两次发射之间至少间隔 300ms

### 3.7 close()

**功能**：关闭视觉门户，释放资源

## 4. 计算原理

### 4.1 目标位置计算

使用以下公式计算目标的相对位置：

```
relative_x = deltaH * cot(yaw) * cos(roll)
relative_y = deltaH * cot(yaw) * sin(roll)
```

其中：
- `deltaH`：炮口与目标的高度差（米）
- `yaw`：仰角（度）
- `roll`：水平旋转角（度）

### 4.2 发射参数计算

使用 RK4 算法计算发射参数：

1. **高度差设置**：通过 `autoSelect.setDeltaH(deltaH)` 设置
2. **初始参数**：
   - 初始初速度：从 Shooter 类获取当前速度，并使用 k 和 b 转换
   - 初始仰角：使用传入的 yaw 参数
3. **轨迹计算**：调用 `autoSelect.Select()` 计算最优发射参数

### 4.3 速度转换

使用以下公式进行速度转换：

```
speed = k * v0 + b  // 物理速度转电机转速
v0 = (speed - b) / k  // 电机转速转物理速度
```

其中：
- `v0`：物理速度（m/s）
- `speed`：电机转速（度/秒）
- `k`、`b`：速度转换参数

## 5. 代码优化建议

### 5.1 硬件参数调整

- **YAW_TICKS_PER_DEGREE**：根据实际电机编码器参数调整
- **YAW_MAX_POWER**：根据电机性能调整
- **APRILTAG_ANGLE_TOLERANCE**：根据视觉识别精度调整

### 5.2 计算优化

- **deltaH**：建议根据实际硬件测量值设置，而非使用默认值 0.5
- **初始初速度**：可以根据目标距离动态调整
- **发射机构**：需要根据实际硬件实现触发逻辑

### 5.3 错误处理

- 添加对硬件初始化失败的处理
- 添加对 AprilTag 检测失败的处理
- 添加对 RK4 计算失败的处理

## 6. 使用示例

```java
// 初始化 Turret
Turret turret = new Turret(hardwareMap, telemetry);

// 设置速度转换参数
turret.set(100, 50); // 示例参数，需要根据实际硬件校准

// 自动瞄准并发射
double[] targetAngles = turret.aim();
turret.shoot(targetAngles[0], targetAngles[1]);

// 每帧更新
@Override
public void loop() {
    boolean shouldShoot = gamepad1.a; // 示例：按下 A 键发射
    turret.update(shouldShoot);
}

// 结束时关闭视觉门户
@Override
public void stop() {
    turret.close();
}
```

## 7. 常见问题

### 7.1 AprilTag 识别失败

- 检查摄像头位置和角度
- 确保 AprilTag 清晰可见
- 调整摄像头曝光和焦距

### 7.2 发射精度问题

- 校准速度转换参数 k 和 b
- 确保 deltaH 设置正确
- 检查发射轮是否磨损

### 7.3 电机控制不稳定

- 调整 PID 参数（在 Shooter 类中）
- 确保电机连接牢固
- 检查电源电压是否稳定

## 8. 总结

Turret 类提供了一个完整的炮台控制系统，集成了视觉识别、角度控制和轨迹计算功能。通过合理配置硬件参数和调整计算参数，可以实现高精度的自动瞄准和发射。

使用时，需要根据实际硬件情况调整相关参数，确保系统稳定可靠地运行。