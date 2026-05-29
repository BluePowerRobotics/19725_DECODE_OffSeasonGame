        package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 * todo标签表示该值未测定
 */
public class HypParams {
    /**
     * 机器人碰撞框（单位：英寸）
     * 定义机器人在场地中的碰撞边界，用于避障和边界检测
     */
    public static ConvexPolygon BoundingBox = new ConvexPolygon(
        new Point2D(6.2, 6.7),
        new Point2D(-6.2, 6.7),
        new Point2D(-6.2, -6.7),
        new Point2D(6.2, -6.7)
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
     * todo:炮口与目标的高度差（单位：米）
     * 用于RK4弹道计算，即发射点与目标点之间的垂直距离
     */
    //public static double deltaH = 0.9;
    /**
     * todo:AprilTag与炮口的高度差（单位：英寸）
     */
    public static double TagH= 33;
    /**
     * todo:相机仰角（单位：度）
     */
    public static double WebcamTheta = 45;
    /**
     * todo:反转炮台水平电机时需要额外转过的角（单位：度）
     */
    public static double ReverseRollAngle = 2;

    /**
     * todo:发射前等待时间（单位：ms）
     * 飞轮开始加速前的准备等待时间，此期间sweeper反转PrepareAngle
     */
    public static int WaitTime = 500;

    /**
     * todo:发射前sweeper反转角度（单位：tick）
     * sweeper在发射前反向旋转的编码器ticks数，将球拉离飞轮
     */
    public static int PrepareAngle = 100;

    /**
     * todo:sweeper吃球速度（单位：tick/s）
     */
    public static int SweeperEatVel = 1960;

    /**
     * todo:sweeper给出artifact速度（单位：tick/s）
     */
    public static int SweeperGiveArtifactVel = 1960;

    /**
     * todo:sweeper吐球速度（单位：tick/s）
     */
    public static int SweeperOutputVel = -960;

    /**
     * todo:sweeper发射触发速度（单位：tick/s）
     */
    public static int SweeperTriggerVel = 3000;

    /**
     * todo:机器人最大线速度（单位：英寸/秒）
     * 底盘运动时允许的最大平移速度
     */
    public static double maxV = 2;

    /**
     * todo:机器人最大角速度（单位：弧度/秒）
     * 底盘旋转时允许的最大角速度
     */
    public static double maxOmega = Math.PI;

    /**
     * todo:红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(0, 0, 0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(0, 0, 0);

    /**
     * todo:自动模式下游走速度（单位：米/秒）
     * 机器人在自动搜索状态下的移动速度
     */
    public static double WanderSpeed = 1;
    /*
     * 最大小球偏移角（单位：弧度）
     * 可以根据limelight的视场角和实际情况调整
     */
    public static double MaxBearing=Math.toRadians(25);

    /**
     * 初始操控模式标志
     * true = 无头模式(field-centric)：摇杆控制场地坐标系运动，机器人自动旋转补偿
     * false = 有头模式(robot-centric)：摇杆控制机器人本体坐标系运动（默认）
     */
    public static boolean InitialUseNoHeadMode = false;

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
     * todo:目标确认所需的连续帧数
     * 一个候选目标需要连续出现这么多帧才能被确认为有效目标
     */
    public static int confirmationFrames = 10;

    /**
     * todo:目标移除所需的连续缺失帧数
     * 一个已确认的目标需要连续缺失这么多帧才会被移除
     */
    public static int removalFrames = 30;

    /**
     * todo:低通滤波系数
     * 用于Tracker中对目标角度的滤波处理，值越小滤波越平滑
     */
    public static double FilterAlpha = 0.3;

    /**
     * todo:球满时与距离传感器的距离（单位：mm）
     * 传感器检测到小于此值时认为球已满
     */
    public static double maxdistance = 20;

    /**
     * todo:球空时与距离传感器的距离（单位：mm）
     * 传感器检测到大于此值时认为球已空
     */
    public static double mindistance = 50;

    /**
     * todo:距离传感器异常阈值（单位：mmm）
     * 传感器在小于此距离时读数不稳定，需要过滤
     */
    public static double wrongdistance = 10;

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

    /**
     * todo:红队吃球位姿列表（单位：英寸，弧度）
     * 包含多个预设的吃球位置，机器人会按顺序访问这些位置
     */
    public static Pose2d[] EatPosesRed = {
            new Pose2d(10, 20, Math.toRadians(90)),
            new Pose2d(20, 20, Math.toRadians(90))
    };

    /**
     * todo:蓝队吃球位姿列表（单位：英寸，弧度）
     * 包含多个预设的吃球位置，机器人会按顺序访问这些位置
     */
    public static Pose2d[] EatPosesBlue = {
            new Pose2d(10, -20, Math.toRadians(-90)),
            new Pose2d(20, -20, Math.toRadians(-90))
    };
}