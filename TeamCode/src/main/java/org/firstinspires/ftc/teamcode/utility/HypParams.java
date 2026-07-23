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
        new Point2D(6.8, 4.8),
        new Point2D(-6.8, 4.8),
        new Point2D(-6.8, -4.8),
        new Point2D(6.8, -4.8)
    );

    /**
     * 左方射击区域（单位：英寸）
     * 机器人在此区域内可以进行射击
     */
    public static ConvexPolygon SHOOTING_AREA_LEFT = new ConvexPolygon(
        new Point2D(0, 0),
        new Point2D(-72, 72),
        new Point2D(-72, -72)
    );

    /**
     * 右方射击区域（单位：英寸）
     * 机器人在此区域内可以进行射击
     */
    public static ConvexPolygon SHOOTING_AREA_RIGHT = new ConvexPolygon(
        new Point2D(72, 24),
        new Point2D(72, -24),
        new Point2D(48, 0)
    );
    //是否去小三角
    public static ConvexPolygon SHOOTING_FARAREA_LEFT = new ConvexPolygon(
            new Point2D(0, 0),
            new Point2D(-72, 72),
            new Point2D(-72, -72)
    );
    public static ConvexPolygon SHOOTING_FARAREA_RIGHT = new ConvexPolygon(
            new Point2D(72, 24),
            new Point2D(72, -24),
            new Point2D(48, 0)
    );
    public static boolean ToLeft=true;
    /**
     * todo:炮口与目标的高度差（单位：米）
     * 用于RK4弹道计算，即发射点与目标点之间的垂直距离
     */
    //public static double deltaH = 0.9;
    /**
     * todo:AprilTag与炮口的高度差（单位：英寸）
     */
    public static double TagH= 17;
    /**
     * todo:相机仰角（单位：度）
     */
    public static double WebcamTheta = 20;

    /**
     * todo:相机在地面投影与车基准点的水平距离（单位：英寸）
     * 相机位于车基准点正前方该距离处，用于校正视觉瞄准时的水平距离计算
     */
    public static double WebCamCenterDistance = 7;
        ;
    /**
     * todo:射程最远时所对仰角
     */
    public static double BestYaw = 50;
    /**
     * 炮台水平旋转角度范围（单位：度）
     * 炮台实际可旋转的机械角度限制为 [-maxRoll, maxRoll]
     */
    public static double maxRoll = 110;
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
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);
    /**
     * 红队远距离起始姿态（单位：英寸，弧度）
     */
    public static Pose2d StartPoseFarRed = new Pose2d(64.2, 29.4, Math.PI);
    /**
     * 蓝队远距离起始姿态（单位：英寸，弧度）
     */
    public static Pose2d StartPoseFarBlue = new Pose2d(64.2, -29.4, Math.PI);
    /**
     * todo:红队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);
    /**
     * todo:蓝队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);
    /*
        * 最大小球偏移角（单位：角度）
        * 可以根据limelight的视场角和实际情况调整
     */
    public static double MaxBearing=Math.toRadians(26);

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
     * todo:EmptySensor处球的HSV范围下限
     * 数组：[H, S, V]，H范围0~360，S/V范围0~1
     */
    public static float[] Ball_Empty_HSV_MIN = {0f, 0.08f, 0.08f};

    /**
     * todo:EmptySensor处球的HSV范围上限
     * 数组：[H, S, V]
     */
    public static float[] Ball_Empty_HSV_MAX = {360f, 0.92f, 1f};

    /**
     * todo:FullSensor处球的HSV范围下限
     * 数组：[H, S, V]，H范围0~360，S/V范围0~1
     */
    public static float[] Ball_Full_HSV_MIN = {0f, 0.1f, 0.01f};

    /**
     * todo:FullSensor处球的HSV范围上限
     * 数组：[H, S, V]
     */
    public static float[] Ball_Full_HSV_MAX = {360f, 0.95f, 0.95f};
    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);

    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);
    public static Pose2d StopPoseFarBlue = new Pose2d(48, -36, Math.PI);

    public static Pose2d StopPoseFarRed = new Pose2d(48, 36, Math.PI);

    // ======== AutoAction 自动阶段参数 ========

    /**
     * todo:红队搜索位姿（单位：英寸，弧度）
     * 前往此位置等待Limelight检测到足够的球
     */
    public static Pose2d searchPoseRed = new Pose2d(63, 63, Math.PI);

    /**
     * todo:蓝队搜索位姿（单位：英寸，弧度）
     */
    public static Pose2d searchPoseBlue = new Pose2d(63, -63, Math.PI);

    /**
     * todo:吃球移动距离（单位：英寸）
     * EatAction中intake激活时沿Y轴移动的距离
     */
    public static double EatDistance = 24;

    /**
     * todo:吃球等待时间（单位：秒）
     * 到达吃球位置后保持intake开启的时间
     */
    public static double EatSecond = 2;

    /**
     * todo:停车时间阈值（单位：毫秒）
     * 自动阶段剩余时间小于此值时执行停车
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * 自动阶段总时长（单位：毫秒）
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;

    /**
     * 红队目标 AprilTag ID
     */
    public static int targetTagIdRed = 24;

    /**
     * 蓝队目标 AprilTag ID
     */
    public static int targetTagIdBlue = 20;

    /**
     * 判断HSV值是否落在EmptySensor处球的预设范围内
     *
     * @param hsv HSV数组，[H, S, V]
     * @return true 表示该颜色匹配EmptySensor处球
     */
    public static boolean isBall_Empty(float[] hsv) {
        return hsv[0] >= Ball_Empty_HSV_MIN[0] && hsv[0] <= Ball_Empty_HSV_MAX[0]
            && hsv[1] >= Ball_Empty_HSV_MIN[1] && hsv[1] <= Ball_Empty_HSV_MAX[1]
            && hsv[2] >= Ball_Empty_HSV_MIN[2] && hsv[2] <= Ball_Empty_HSV_MAX[2];
    }

    /**
     * 判断HSV值是否落在紫色球的预设范围内
     *
     * @param hsv HSV数组，[H, S, V]
     * @return true 表示该颜色匹配紫色球
     */
    public static boolean isBall_Full(float[] hsv) {
        return hsv[0] >= Ball_Full_HSV_MIN[0] && hsv[0] <= Ball_Full_HSV_MAX[0]
            && hsv[1] >= Ball_Full_HSV_MIN[1] && hsv[1] <= Ball_Full_HSV_MAX[1]
            && hsv[2] >= Ball_Full_HSV_MIN[2] && hsv[2] <= Ball_Full_HSV_MAX[2];
    }


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
     * 预设飞轮最大转速（RPM）
     * 用于 TeleOp 中 right_trigger 控制预载速度的上限
     */
    public static int maxPreSpeed = 1800;

    /**
     * 反转模式下飞轮转速（RPM，负值表示反转）
     * 当 P1 右 Bumper 按下时，飞轮以该速度反转排球
     */
    public static int shooterReverseSpeed = -1800;

    /**
     * trigger舵机发射位置（0~1）
     * 舵机旋转到此位置时发射球
     */
    public static double triggerLaunchPosition = 0.679;

    /**
     * trigger舵机复位位置（0~1）
     * 发射后舵机回到此位置
     */
    public static double triggerResetPosition = 0.421;

    /**
     * todo:红队吃球位姿列表（单位：英寸，弧度）
     * 包含多个预设的吃球位置，机器人会按顺序访问这些位置
     */
    public static Pose2d[] EatPosesRed = {
        new Pose2d(-12, 24, Math.toRadians(90)),
        new Pose2d(12, 24, Math.toRadians(90)),
        new Pose2d(36, 24, Math.toRadians(90)),
    };

    /**
     * todo:蓝队吃球位姿列表（单位：英寸，弧度）
     * 包含多个预设的吃球位置，机器人会按顺序访问这些位置
     */
    public static Pose2d[] EatPosesBlue = {
        new Pose2d(-12, -24, Math.toRadians(-90)),
        new Pose2d(12, -24, Math.toRadians(-90)),
        new Pose2d(36, -24, Math.toRadians(90)),
    };

    /**
     * todo:红队远距离吃球位姿（单位：英寸，弧度）
     * AutoActionRed中从StartPoseFar出发前往此位置吃球
     */
    public static Pose2d EatPoseFarRed = new Pose2d(63, 36.7, Math.PI/2);
    public static Pose2d[] EatPoseRed_FAR = {
        new Pose2d(36, 24, Math.toRadians(90)),
        new Pose2d(63, 36.7, Math.PI/2),
    };

    /**
     * todo:蓝队远距离吃球位姿（单位：英寸，弧度）
     * AutoActionBlue中从StartPoseFar出发前往此位置吃球
     */
    public static Pose2d EatPoseFarBlue = new Pose2d(63, -36.7,- Math.PI/2);
    public static Pose2d[] EatPosesBlue_FAR = {
            new Pose2d(36, -24, Math.toRadians(-90)),
            new Pose2d(63, -36.7, Math.PI/2),
    };

    /**
     * 红队密道区位姿（单位：英寸，弧度）
     * 用于自动阶段前往密道区使用Limelight自主吃球
     */
    public static Pose2d TunnelPoseRed = new Pose2d(24, 48, Math.PI);

    /**
     * 蓝队密道区位姿（单位：英寸，弧度）
     * 用于自动阶段前往密道区使用Limelight自主吃球
     */
    public static Pose2d TunnelPoseBlue = new Pose2d(24, -48, Math.PI);

    /**
     * 密道区自主吃球持续时间（单位：毫秒）
     */
    public static long LIMELIGHT_EAT_DURATION_MS = 8000;

    /**
     * 自动阶段等待时间（单位：毫秒）
     * 用于吃球后等待5秒
     */
    public static long AUTO_WAIT_DURATION_MS = 5000;
}