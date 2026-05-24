package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
@Config
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
        new Point2D(48, 0)
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
    public static double deltaH = 0.9;
    /**
     * AprilTag与炮口的高度差（单位：米）
     */
    public static double TagH=0.85;
    /**
     * 相机与炮口的水平距离（单位：米）
     */
    public static double WebcamR = 0.1;
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
    public static double WanderSpeed = 0.2;
    /*
        * 最大小球偏移角（单位：弧度）
        * 可以根据limelight的视场角和实际情况调整
     */
    public static double MaxBearing=Math.toRadians(25);

    /**
     * 初始操控模式标志
     * false表示使用"有头模式"（场心地坐标系），true表示使用"无头模式"（机器人坐标系）
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
    public static double BearingThreshold = 0.35; //limelight横向视场角的约一半

    /**
     * 目标确认所需的连续帧数
     * 一个候选目标需要连续出现这么多帧才能被确认为有效目标
     */
    public static int confirmationFrames = 10;

    /**
     * 目标移除所需的连续缺失帧数
     * 一个已确认的目标需要连续缺失这么多帧才会被移除
     */
    public static int removalFrames = 30;

    /**
     * 低通滤波系数
     * 用于Tracker中对目标角度的滤波处理，值越小滤波越平滑
     */
    public static double FilterAlpha = 0.2;

    /**
     * 球满时与距离传感器的距离（单位：cm）
     * 传感器检测到小于此值时认为球已满
     */
    public static double maxdistance = 20;

    /**
     * 球空时与距离传感器的距离（单位：cm）
     * 传感器检测到大于此值时认为球已空
     */
    public static double mindistance = 50;

    /**
     * 距离传感器异常阈值（单位：cm）
     * 传感器在小于此距离时读数不稳定，需要过滤
     */
    public static double wrongdistance = 30;

    /**
     * 根据 AprilTag ID 获取球门位置
     * @param id AprilTag ID
     * @return {x, y} 坐标数组，如果找不到返回 null
     */
    public static double[] getGoalPosition(int id) {
        if (id==20) {
            return new double[]{-72, -72};
        }else if (id==24){
            return new double[]{-72, 72};
        }
        return null;
    }
}