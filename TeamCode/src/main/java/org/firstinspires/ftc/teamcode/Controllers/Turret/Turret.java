package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.RK4.AutoSelect;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;

import java.util.List;

public class Turret {
    public Shooter shooter;
    Telemetry telemetry;
    private TurretDegreeController turretDegreeController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double roll;
    private double yaw;
    private double k;
    private double b;
    private double delta_H;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        roll = 0.0;
        yaw = 0.0;
        this.k = 1.0;
        this.b = 0.0;
        this.delta_H = HypParams.deltaH;

        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry = telemetry;

        turretDegreeController = new TurretDegreeController(hardwareMap, telemetry);

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
            targetRoll = this.roll + bearing;
            targetYaw = this.yaw + elevation;
        } else {
            double[] goalPos = HypParams.getGoalPosition(targetTagId);
            if (goalPos != null) {
                double robotX = org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().getX();
                double robotY = org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().getY();
                double robotTheta = org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().getTheta();
                
                double dx = goalPos[0] - robotX;
                double dy = goalPos[1] - robotY;
                
                double angleToGoal = Math.atan2(dy, dx);
                targetRoll = Math.toDegrees(angleToGoal - robotTheta);
                targetYaw = Math.toDegrees(Math.atan2(delta_H, Math.hypot(dx, dy)));
            } else {
                targetRoll = this.roll + 90;
                targetYaw = this.yaw;
            }
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
                launch(); 
            }
        }
    }

    public void launch() {
        //todo: 一次射一个球
    }

    public void update(boolean shouldAim, boolean shouldShoot,int targetTagId) {
        shooter.update();
        turretDegreeController.update();
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

    public void update(double roll, double yaw, int speed, boolean shouldShoot) {
        shooter.update();
        turretDegreeController.update();
        turretDegreeController.rotateTo(roll, yaw);
        if (shouldShoot) {
            if(shooter.setTargetSpeed(speed)){
                telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
                telemetry.update();
                launch(); 
            }
        }
    }
    //只用到标定，主程序不用。


    public void stop() {
        turretDegreeController.stop();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
