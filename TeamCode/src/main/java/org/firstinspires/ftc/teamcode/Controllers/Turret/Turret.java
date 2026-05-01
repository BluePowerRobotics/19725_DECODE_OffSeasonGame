package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.RK4.AutoSelect;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;

import java.util.List;

public class Turret {
    // 核心组件
    public Shooter shooter;                   // 发射系统
    Telemetry telemetry;
    private TurretDegreeController turretDegreeController; // 炮台旋转控制器
    private AprilTagProcessor aprilTag;         // AprilTag处理器
    private VisionPortal visionPortal;          // 视觉门户
    //tagID已经被yyc改成外部传入了，这里没删干净

    // 状态变量
    private double roll;                        // 绕z轴角（水平旋转角）
    private double yaw;                         // 绕y轴角（仰角）
    private double k;                           // 速度转换参数k
    private double b;                           // 速度转换参数b
    private double delta_H;                      // 炮口与目标的高度差

    // 发射机构状态
    private static final double SERVO_REST_POSITION = 0.0;    // 伺服电机休息位置
    private static final double SERVO_LAUNCH_POSITION = 0.5;  // 伺服电机发射位置

    // 常量
    private static final double APRILTAG_ANGLE_TOLERANCE = 0.5;      // AprilTag瞄准容差（度）

    // 构造函数
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, double k,double b,double delta_H) {
        roll = 0.0;
        yaw = 0.0;
        this.k = k;
        this.b = b;
        this.delta_H = delta_H;

        // 初始化发射系统
        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry=telemetry;

        // 初始化炮台旋转控制器
        turretDegreeController = new TurretDegreeController(hardwareMap,telemetry);

        initAprilTag(hardwareMap);

    }

    /**
     * 获取当前高度差
     * @return 当前高度差
     */
    public double getDeltaH() {
        return delta_H;
    }

    // 初始化AprilTag处理器
    private void initAprilTag(HardwareMap hardwareMap) {
        // 创建AprilTag处理器
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 创建视觉门户
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /**
     * 旋转到指定角度
     * @param roll 水平旋转角（度，逆时针为正）
     * @param yaw 仰角（度）
     * @return 是否成功
     */
    public boolean rotate_to(double roll, double yaw){
        boolean success = turretDegreeController.rotateTo(roll, yaw);
        if (success) {
            // 旋转成功后更新内部状态
            this.roll = roll;
            this.yaw = yaw;
        }
        return success;
    }

    public double[] get_angle() {
        double[] angles = turretDegreeController.get_angle();
        roll = angles[0];
        yaw = angles[1];
        return new double[]{roll, yaw};
    }

    public Object[] aim(int targetTagId) {
        get_angle();

        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean isTargetFound = false;
        AprilTagDetection targetDetection = null;


        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                targetDetection = detection;
                isTargetFound = true;
                break;
            }
        }

        double targetRoll;
        double targetYaw;

        if (isTargetFound && targetDetection != null) {
            double bearing = targetDetection.ftcPose.bearing;
            double elevation = targetDetection.ftcPose.elevation;
            //TODO check +-
            targetRoll = this.roll + bearing;
            //TODO ???
            targetYaw = this.yaw + elevation;
        } else {
            //TODO check logic
            targetRoll = this.roll + 90;
            targetYaw = this.yaw;
        }
        //先这样写，但可能apriltag的实际位置低于球门，yaw要加一个常量
        rotate_to(targetRoll, targetYaw);

        double[] angles = get_angle();
        return new Object[]{isTargetFound, angles[0], angles[1]};
    }

    public void set(double k, double b) {
        this.k = k;
        this.b = b;
    }

    public void shoot(double roll, double yaw) {
        double deltaH = delta_H;
        double cotYaw = 1.0 / Math.tan(Math.toRadians(yaw));
        double targetX = deltaH * cotYaw * Math.cos(Math.toRadians(roll));
        double targetY = deltaH * cotYaw * Math.sin(Math.toRadians(roll));

        AutoSelect autoSelect = new AutoSelect();
        autoSelect.setDeltaH(deltaH);

        double currentSpeed = shooter.getCurrentVelocity();
        double initialV0 = (k != 0) ? (currentSpeed - b) / k : 8.0;
        double initialTheta = Math.toRadians(yaw);
        AutoSelect.AutoSelectResult result = autoSelect.Select(targetX, targetY, 0, 0, initialV0, initialTheta);

        if (result.success) {
            double v0 = result.v0;
            int speed = (int) (k * v0 + b);
            if(shooter.setTargetSpeed(speed)){
                telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
                telemetry.update();
                //shooter.launch(); //launch()要把3个球射完
                //为什么把扳机控制删了？
                //你们有真的有扳机吗？？？？
            }
        }
    }

    public void update(boolean shouldAim, boolean shouldShoot,int targetTagId) {
        shooter.update();

        if (shouldAim) {
            Object[] aimResult = aim(targetTagId);
            boolean isTargetFound = (boolean) aimResult[0];
            double targetRoll = (double) aimResult[1];
            double targetYaw = (double) aimResult[2];

            if (shouldShoot && isTargetFound) {
                shoot(targetRoll, targetYaw);
            }
        } else if (shouldShoot) {
            shoot(roll, yaw);//注意逻辑：Turret.shoot传入的是目标位置对应的仰角，实际发射不使用传入的参数
        }
    }


    public void stop() {
        turretDegreeController.stop();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
