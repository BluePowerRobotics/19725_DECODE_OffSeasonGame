package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
public class HypParams {
    /**
     * 机器人碰撞框（单位：英寸）
     * 定义机器人在场地中的碰撞边界，用于避障和边界检测
     */
    public static ConvexPolygon BoundingBox = new ConvexPolygon(
        new Point2D(20, 20),
        new Point2D(-20, 20),
        new Point2D(-20, -20),
        new Point2D(20, -20)
    );

    /**
     * 左方射击区域（单位：英寸）
     * 机器人在此区域内可以进行射击
     */
    public static ConvexPolygon SHOOTING_AREA_LEFT = new ConvexPolygon(
        new Point2D(72, 24),
        new Point2D(72, -24),
        new Point2D(-48, 0)
    );

    /**
     * 右方射击区域（单位：英寸）
     * 机器人在此区域内可以进行射击
     */
    public static ConvexPolygon SHOOTING_AREA_RIGHT = new ConvexPolygon(
        new Point2D(0, 0),
        new Point2D(-72, 72),
        new Point2D(-72, -72)
    );
    //是否去小三角
    public static boolean ToLeft=true;
    /**
     * 炮口与目标的高度差（单位：米）
     * 用于RK4弹道计算，即发射点与目标点之间的垂直距离
     */
    public static double deltaH = 0.5;

    /**
     * 机器人最大线速度（单位：米/秒）
     * 底盘运动时允许的最大平移速度
     */
    public static double maxV = 0.5;

    /**
     * 机器人最大角速度（单位：弧度/秒）
     * 底盘旋转时允许的最大角速度
     */
    public static double maxOmega = Math.PI / 2;

    /**
     * 红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(0, 0, 0);

    /**
     * 蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(0, 0, 0);

    /**
     * 自动模式下游走速度（单位：米/秒）
     * 机器人在自动搜索状态下的移动速度
     */
    public static double WanderSpeed = 1.0;

    /**
     * Limelight摄像头安装高度（单位：米）
     * 即Projector中的h参数，用于目标坐标计算
     */
    public static double Limelight_h = 0.0;

    /**
     * Limelight摄像头水平偏移量（单位：米）
     * 即Projector中的m0参数，用于目标坐标计算
     */
    public static double Limelight_m0 = 0.0;

    /**
     * 初始操控模式标志
     * true表示使用"有头模式"（场心地坐标系），false表示使用"无头模式"（机器人坐标系）
     */
    public static boolean InitialRunningToPose = true;

    /**
     * 球的质量（单位：千克）
     * 用于RK4弹道动力学计算
     */
    public static double ballMass = 0.06;

    /**
     * 目标角度每帧变化范围（单位：弧度）
     */
    public static double BearingThreshold = 0.15;

    /**
     * 目标确认所需的连续帧数
     * 一个候选目标需要连续出现这么多帧才能被确认为有效目标
     */
    public static int confirmationFrames = 3;

    /**
     * 目标移除所需的连续缺失帧数
     * 一个已确认的目标需要连续缺失这么多帧才会被移除
     */
    public static int removalFrames = 5;

    /**
     * 低通滤波系数
     * 用于Tracker中对目标角度的滤波处理，值越小滤波越平滑
     */
    public static double FilterAlpha = 0.2;
}