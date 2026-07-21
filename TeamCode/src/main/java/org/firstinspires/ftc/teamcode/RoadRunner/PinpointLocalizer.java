package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * 使用 GoBilda Pinpoint 驱动的定位器实现
 * Pinpoint 是一个集成了编码器和 IMU 的定位系统
 */
@Config
public final class PinpointLocalizer implements Localizer {
    /**
     * Pinpoint 定位器的参数配置类
     */
    public static class Params {
        public double parYTicks =  2460.9120957820664; // 平行编码器的 y 位置（tick 单位）
        public double perpXTicks = 1907.1003263946116; // 垂直编码器的 x 位置（tick 单位）
        public double pinXOffset = 0; // Pinpoint 基准点相对于机器人中心的 X 偏移（英寸，+为前方）
        public double pinYOffset = 0; //
        // Pinpoint 基准点相对于机器人中心的 Y 偏移（英寸，+为左侧）
    }

    /**
     * 全局参数实例，可通过 FTC Dashboard 实时调整
     */
    public static Params PARAMS = new Params();

    /**
     * GoBilda Pinpoint 驱动实例
     */
    public final GoBildaPinpointDriver driver;
    
    /**
     * 初始平行编码器方向和初始垂直编码器方向
     */
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    /**
     * 世界坐标系到机器人起始位置的变换（初始位姿偏移）
     */
    private Pose2d txWorldRobotInitial;
    
    /**
     * Pinpoint 坐标系到机器人坐标系的固定变换
     * 表示 Pinpoint 基准点相对于机器人中心的位置偏移
     * 例如：Pinpoint 安装在机器人中心前方 4 英寸、左侧 2 英寸处，则为 new Pose2d(4, 2, 0)
     * 注意：此偏移量以机器人坐标系为参考（+X 前，+Y 左）
     */
    private final Pose2d txPinpointRobot;
    
    /**
     * Pinpoint 报告的当前位置（相对于其初始化位置）
     */
    private Pose2d pinpointReportedPose = new Pose2d(0, 0, 0);

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param inPerTick 每个编码器 tick 对应的英寸数
     * @param initialPose 初始位姿
     */
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);

        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        // 初始化 Pinpoint 相对于机器人中心的固定偏移量
        txPinpointRobot = new Pose2d(PARAMS.pinXOffset, PARAMS.pinYOffset, 0);
        // 记录机器人初始世界位姿
        txWorldRobotInitial = initialPose;
    }

    /**
     * 设置当前位姿
     * 当通过 AprilTag 等外部信息知道机器人当前位置时调用
     * @param pose 机器人当前的世界位姿
     */
    @Override
    public void setPose(Pose2d pose) {
        // 计算 Pinpoint 的初始世界位姿 = 机器人当前位姿 * Pinpoint相对机器人偏移
        // 然后反向算出 Pinpoint 报告的位置应是多少
        // pinpointReportedPose = (初始Pinpoint世界位姿)⁻¹ * 当前Pinpoint世界位姿
        // 但由于我们直接设置，我们更新 txWorldRobotInitial 使得 getPose() 返回 pose
        Pose2d currentPinpointWorld = pose.times(txPinpointRobot);
        // 从 Pinpoint 当前世界位姿反推初始位姿
        // currentPinpointWorld = txWorldRobotInitial * txPinpointRobot * pinpointReportedPose
        // 需要结合当前 pinpointReportedPose 来更新 txWorldRobotInitial
        // 简化处理：直接重置 Pinpoint 的 internal position
        driver.setPosition(new Pose2D(DistanceUnit.INCH, 
            currentPinpointWorld.position.x, currentPinpointWorld.position.y,
            AngleUnit.RADIANS, currentPinpointWorld.heading.log()));
        // 同时更新本地记录
        txWorldRobotInitial = pose;
        pinpointReportedPose = new Pose2d(0, 0, 0);
    }

    /**
     * 获取当前在世界坐标系中的位姿
     * @return 机器人中心的当前世界位姿
     */
    @Override
    public Pose2d getPose() {
        // 1. 计算 Pinpoint 的初始世界位姿
        Pose2d initialPinpointWorld = txWorldRobotInitial.times(txPinpointRobot);
        
        // 2. Pinpoint 报告的位置是在初始航向坐标系下的位移
        //   需要旋转到世界坐标系
        double initH = txWorldRobotInitial.heading.log();
        double cosInit = Math.cos(initH);
        double sinInit = Math.sin(initH);
        double dispWorldX = pinpointReportedPose.position.x * cosInit - pinpointReportedPose.position.y * sinInit;
        double dispWorldY = pinpointReportedPose.position.x * sinInit + pinpointReportedPose.position.y * cosInit;
        
        // 3. Pinpoint 的当前世界位置
        double pinWorldX = initialPinpointWorld.position.x + dispWorldX;
        double pinWorldY = initialPinpointWorld.position.y + dispWorldY;
        double heading = pinpointReportedPose.heading.log(); // 绝对航向（来自IMU）
        
        // 4. 减去 Pinpoint 相对于机器人中心的偏移（旋转到当前航向）
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double robotX = pinWorldX - (txPinpointRobot.position.x * cosH - txPinpointRobot.position.y * sinH);
        double robotY = pinWorldY - (txPinpointRobot.position.x * sinH + txPinpointRobot.position.y * cosH);
        
        return new Pose2d(robotX, robotY, heading);
    }

    /**
     * 更新位姿估计
     * @return 当前速度估计
     */
    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            // 读取 Pinpoint 报告的当前位置（相对于其初始化位置和初始航向）
            pinpointReportedPose = new Pose2d(
                driver.getPosX(DistanceUnit.INCH), 
                driver.getPosY(DistanceUnit.INCH), 
                driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            );
            
            // 计算世界坐标系下的速度
            Vector2d worldVelocity = new Vector2d(
                driver.getVelX(DistanceUnit.INCH), 
                driver.getVelY(DistanceUnit.INCH)
            );
            
            // 将世界坐标系速度转换为机器人坐标系速度
            double heading = pinpointReportedPose.heading.log();
            Vector2d robotVelocity = Rotation2d.fromDouble(-heading).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
