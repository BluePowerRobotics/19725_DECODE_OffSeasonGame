package org.firstinspires.ftc.teamcode.Controllers.Turret;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.BST.BSTsolver;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Turret {
    public Shooter shooter;
    Telemetry telemetry;
    public TurretDegreeController turretDegreeController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double roll;
    private double yaw;
    private double k;
    private double b;
    private double delta_H;
    private BSTsolver bstSolver;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        roll = 0.0;
        yaw = 0.0;
        this.k = 1.0;
        this.b = 0.0;
        this.delta_H = HypParams.deltaH;

        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry = telemetry;

        turretDegreeController = new TurretDegreeController(hardwareMap, telemetry);
        bstSolver = new BSTsolver();

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
        // 定义摄像头位置和方向
        // 位置：摄像头在机器人坐标系中的位置（这里假设在原点）
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, HypParams.WebcamTheta, -90, 0);

        // 创建AprilTag处理器并设置摄像头姿态
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
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
            this.roll = MathSolver.normalizeAngle(roll);
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
            double distance = HypParams.TagH / Math.tan(Math.toRadians(elevation)) + HypParams.WebcamR;
            targetYaw = Math.toDegrees(Math.atan2(HypParams.TagH, distance));
            if(targetRoll <= -180 || targetRoll >= 180) {
                double[] goalPos = HypParams.getGoalPosition(targetTagId);
                if (goalPos != null) {
                    double robotX = RobotPosition.getInstance().getX();
                    double robotY = RobotPosition.getInstance().getY();
                    double robotTheta = RobotPosition.getInstance().getTheta();

                    double dx = goalPos[0] - robotX;
                    double dy = goalPos[1] - robotY;

                    double relativeDx = dx * Math.cos(robotTheta) + dy * Math.sin(robotTheta);
                    double relativeDy = -dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);

                    double theta = Math.atan2(relativeDy, relativeDx);
                    targetRoll = MathSolver.normalizeAngle(Math.toDegrees(theta - robotTheta));
                    targetYaw = Math.toDegrees(Math.atan2(HypParams.TagH, Math.hypot(relativeDx, relativeDy)));
                }
            }
        } else {
            double[] goalPos = HypParams.getGoalPosition(targetTagId);
            if (goalPos != null) {
                double robotX = RobotPosition.getInstance().getX();
                double robotY = RobotPosition.getInstance().getY();
                double robotTheta = RobotPosition.getInstance().getTheta();
                
                double dx = goalPos[0] - robotX;
                double dy = goalPos[1] - robotY;
                
                double relativeDx = dx * Math.cos(robotTheta) + dy * Math.sin(robotTheta);
                double relativeDy = -dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);

                double theta = Math.atan2(relativeDy, relativeDx);
                targetRoll = MathSolver.normalizeAngle(Math.toDegrees(theta - robotTheta));
                targetYaw = Math.toDegrees(Math.atan2(HypParams.TagH, Math.hypot(relativeDx, relativeDy)));
            } else {
                throw new IllegalArgumentException("Goal position not found for targetTagId: " + targetTagId);
            }
        }
        rotate_to(targetRoll, targetYaw);

        double[] angles = get_angle();
        return new Object[]{isTargetFound, angles[0], angles[1]};
    }
    /*
    public void set(double k, double b) {
        this.k = k;
        this.b = b;
    }
    */
    public void shoot(double roll, double yaw) {
        double deltaH = delta_H;
        double cotYaw = 1.0 / Math.tan(Math.toRadians(yaw));
        double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(roll));
        double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(roll));
        BSTsolver.Solution solution = bstSolver.predict(RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(), targetX, targetY);
        if (solution.success) {
            double targetRoll = Math.toDegrees(solution.roll);
            double targetYaw = solution.yaw;
            int targetSpeed = solution.speed;

            if (shooter.setTargetSpeed(targetSpeed) && turretDegreeController.rotateTo(targetRoll, targetYaw)) {
                telemetry.addData("BST Shooting", "yaw= %.2f, roll= %.2f, speed= %d", targetYaw, targetRoll, targetSpeed);
                telemetry.update();
                launch();
            }
        } else {
            telemetry.addData("BST Error", solution.message);
            telemetry.update();
        }
        
    }

    public void shootByPosition(int targetTagId) {
        double[] goalPos = HypParams.getGoalPosition(targetTagId);
        if (goalPos != null) {
            double robotX = RobotPosition.getInstance().getX();
            double robotY = RobotPosition.getInstance().getY();
            double robotTheta = RobotPosition.getInstance().getTheta();
            double vx = RobotPosition.getInstance().getVx();
            double vy = RobotPosition.getInstance().getVy();

            double dx = goalPos[0] - robotX;
            double dy = goalPos[1] - robotY;

            double relativeDx = dx * Math.cos(robotTheta) + dy * Math.sin(robotTheta);
            double relativeDy = -dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);

            BSTsolver.Solution solution = bstSolver.predict(vx, vy, relativeDx, relativeDy);

            if (solution.success) {
                double targetRoll = Math.toDegrees(solution.roll);
                double targetYaw = solution.yaw;
                int targetSpeed = solution.speed;

                if (shooter.setTargetSpeed(targetSpeed) && turretDegreeController.rotateTo(targetRoll, targetYaw)) {
                    telemetry.addData("BST Shooting", "yaw= %.2f, roll= %.2f, speed= %d", targetYaw, targetRoll, targetSpeed);
                    telemetry.update();
                    launch();
                }
            } else {
                telemetry.addData("BST Error", solution.message);
                telemetry.update();
            }
        } else {
            throw new IllegalArgumentException("No goal position found for target ID");
        }
    }

    public void launch() {
        //todo: 打开闸门
    }

    public void reset(){
        //todo: 关闭闸门
    }

    public void update(boolean shouldAim, boolean shouldShoot,int targetTagId) {
        shooter.update();
        turretDegreeController.update();
        if (shouldAim) {
            Object[] aimResult = aim(targetTagId);
            boolean isTargetFound = (boolean) aimResult[0];
            double targetRoll = (double) aimResult[1];
            double targetYaw = (double) aimResult[2];

            if(!shouldShoot){
                reset();
            }
            else if (isTargetFound) {
                shoot(targetRoll, targetYaw);
            }else{
                shootByPosition(targetTagId); //tag不可用时自动降级
                telemetry.addData("Warning", "Response unavailable, auto downgrading");
                telemetry.update();
            }
        } else if (shouldShoot) {
            shootByPosition(targetTagId);
            //不触发视觉实时瞄准（aim）时，仅根据位姿解算插值
        } else {
            reset();
        }
    }
    //重载输入统一
    public void update(int speed, boolean shouldShoot) {
        shooter.update();
        turretDegreeController.update();
        if (shouldShoot) {
            if(shooter.setTargetSpeed(speed)){
                telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
                telemetry.update();
                launch(); 
            }
        }else{
            reset();
        }
    }

    public void update(double kx) {
        shooter.update();
        DcMotorEx rollMotor = turretDegreeController.rollMotor;
        rollMotor.setPower(kx);
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
        }else{
            reset();
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
