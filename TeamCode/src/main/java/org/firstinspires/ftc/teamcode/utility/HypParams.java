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
     * todo:机器人碰撞框（单位：英寸）
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
     * todo:炮口与目标的高度差（单位：米）
     * 用于RK4弹道计算，即发射点与目标点之间的垂直距离
     */
    //public static double deltaH = 0.9;
    /**
     * todo:AprilTag与炮口的高度差（单位：英寸）
     */
    public static double TagH= 33;
    /**
     * todo:相机与炮口的水平距离（单位：米）
     */
    public static double WebcamR = 0.1;
    /**
     * todo:相机仰角（单位：度）
     */
    public static double WebcamTheta = 45;
    /**
     * todo:射程最远时所对仰角
     */
    public static double BestYaw = 50;
    /**
     * 炮台水平旋转角度范围（单位：度）
     * 炮台实际可旋转的机械角度限制为 [-maxRoll, maxRoll]
     */
    public static double maxRoll = 120;
    /**
     * todo:机器人最大线速度（单位：英寸/秒）
     * 底盘运动时允许的最大平移速度
     */
    public static double maxV = 2;

    /**
     * 机器人最大角速度（单位：弧度/秒）
     * 底盘旋转时允许的最大角速度
     */
    public static double maxOmega = 1.3;

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
     * todo:自动模式下游走速度（单位：英寸/秒）
     * 机器人在自动搜索状态下的移动速度
     */
    public static double WanderSpeed = 1;
    /*
        * 最大小球偏移角（单位：弧度）
        * 可以根据limelight的视场角和实际情况调整
     */
    public static double MaxBearing=Math.toRadians(10); //25

    /**
     * 初始操控模式标志
     * true=无头模式（场心地坐标系），false=有头模式（机器人坐标系）
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
    public static double FilterAlpha = 0.4;

    /**
     * todo:颜色低通滤波系数
     * 用于Tracker中对目标角度的滤波处理，值越小滤波越平滑
     */
    public static double ColorAlpha = 0.4;
    /*
     * todo:绿色球的HSV范围下限
     * 数组：[H, S, V]，H范围0~360，S/V范围0~1
     */
    public static float[] GREEN_HSV_MIN = {100f, 0.3f, 0.3f};

    /**
     * todo:绿色球的HSV范围上限
     * 数组：[H, S, V]
     */
    public static float[] GREEN_HSV_MAX = {140f, 1.0f, 1.0f};

    /**
     * todo:紫色球的HSV范围下限
     * 数组：[H, S, V]，H范围0~360，S/V范围0~1
     */
    public static float[] PURPLE_HSV_MIN = {260f, 0.3f, 0.3f};

    /**
     * todo:紫色球的HSV范围上限
     * 数组：[H, S, V]
     */
    public static float[] PURPLE_HSV_MAX = {310f, 1.0f, 1.0f};

    /**
     * 判断HSV值是否落在绿色球的预设范围内
     *
     * @param hsv HSV数组，[H, S, V]
     * @return true 表示该颜色匹配绿色球
     */
    public static boolean isGreenBall(float[] hsv) {
        return hsv[0] >= GREEN_HSV_MIN[0] && hsv[0] <= GREEN_HSV_MAX[0]
            && hsv[1] >= GREEN_HSV_MIN[1] && hsv[1] <= GREEN_HSV_MAX[1]
            && hsv[2] >= GREEN_HSV_MIN[2] && hsv[2] <= GREEN_HSV_MAX[2];
    }

    /**
     * 判断HSV值是否落在紫色球的预设范围内
     *
     * @param hsv HSV数组，[H, S, V]
     * @return true 表示该颜色匹配紫色球
     */
    public static boolean isPurpleBall(float[] hsv) {
        return hsv[0] >= PURPLE_HSV_MIN[0] && hsv[0] <= PURPLE_HSV_MAX[0]
            && hsv[1] >= PURPLE_HSV_MIN[1] && hsv[1] <= PURPLE_HSV_MAX[1]
            && hsv[2] >= PURPLE_HSV_MIN[2] && hsv[2] <= PURPLE_HSV_MAX[2];
    }

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