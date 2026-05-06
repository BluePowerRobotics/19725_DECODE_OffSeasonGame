完成HypParams.java实现一个类用于存储全局超参数，包括：
BoundingBox //小车碰撞框（inch）
SHOOTING_AREA_RIGHT //右方射击区域（inch）
SHOOTING_AREA_LEFT //左方射击区域（inch）
deltaH //炮口与目标高度差（m）（用于turret和RK4计算）
maxV //最大速度（m/s）
maxOmega //最大角速度（rad/s）
startPoseRed //初始姿态Red（Pose2d,inch）
startPoseBlue //初始姿态Blue（Pose2d,inch）
WanderSpeed //Auto游走速度（m/s）
Limelight_h //即Projector中的h（m），用于Tracker
Limelight_m0 //即Projector中的m0（m），用于Tracker
InitialRunningToPose //初始操控模式
distanceThreshold// 聚类距离阈值，用于判断检测是否属于同一目标
confirmationFrames// 目标确认所需的连续帧数
removalFrames// 目标移除所需的连续缺失帧数

完成后修改其他程序，将底层代码（如RK4,Projector）中原来硬编码或由上层传入的参数改为直接从HypParams中获取，上层代码（如Chassis,Turret）实例化底层类时不再需要传入参数（Chassis构造时改为设定队伍颜色）。（即所有超参对上层代码透明，只需要修改HypParams即可）
