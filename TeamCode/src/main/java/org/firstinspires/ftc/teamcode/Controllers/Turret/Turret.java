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
    private BSTsolver bstSolver;

    private boolean shooting = false;
    private boolean waitingForSpeed = false;
    private long speedWaitStartTime = 0;
    private long lastLaunchTime = 0;
    private static final long SPEED_WAIT_TIMEOUT_MS = 500;
    private static final long LAUNCH_COOLDOWN_MS = 300;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        roll = 0.0;
        yaw = 0.0;
        this.k = 1.0;
        this.b = 0.0;

        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry = telemetry;

        turretDegreeController = new TurretDegreeController(hardwareMap, telemetry);
        bstSolver = new BSTsolver();

        initAprilTag(hardwareMap);
    }

    // 初始化AprilTag处理器
    private void initAprilTag(HardwareMap hardwareMap) {
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
            targetYaw = this.yaw + elevation;
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
                    targetRoll = MathSolver.normalizeAngle(Math.toDegrees(theta));
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
                targetRoll = MathSolver.normalizeAngle(Math.toDegrees(theta));
                targetYaw = Math.toDegrees(Math.atan2(HypParams.TagH, Math.hypot(relativeDx, relativeDy)));
            } else {
                throw new IllegalArgumentException("Goal position not found for targetTagId: " + targetTagId);
            }
        }
        rotate_to(targetRoll, targetYaw);
        return new Object[]{isTargetFound, targetRoll, targetYaw-HypParams.WebcamTheta};
    }
    /*
    public void set(double k, double b) {
        this.k = k;
        this.b = b;
    }
    */
    public void shoot(double roll, double yaw) {
        shooting = true;
        double cotYaw = 1.0 / Math.tan(Math.toRadians(yaw));
        double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(roll));
        double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(roll));
        BSTsolver.Solution solution = bstSolver.predict(RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(), targetX, targetY);
        if (solution.success) {
            double targetRoll = Math.toDegrees(solution.roll);
            double targetYaw = solution.yaw;
            int targetSpeed = solution.speed;

            shooter.setTargetSpeed(targetSpeed);
            turretDegreeController.rotateTo(targetRoll, targetYaw);
            waitingForSpeed = true;
            speedWaitStartTime = System.currentTimeMillis();
            telemetry.addData("BST Shooting", "yaw= %.2f, roll= %.2f, speed= %d", targetYaw, targetRoll, targetSpeed);
            telemetry.update();
        } else {
            telemetry.addData("BST Error", solution.message);
            telemetry.update();
        }
    }

    public void shootByPosition(int targetTagId) {
        shooting = true;
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

                shooter.setTargetSpeed(targetSpeed);
                turretDegreeController.rotateTo(targetRoll, targetYaw);
                waitingForSpeed = true;
                speedWaitStartTime = System.currentTimeMillis();
                telemetry.addData("BST Shooting", "yaw= %.2f, roll= %.2f, speed= %d", targetYaw, targetRoll, targetSpeed);
                telemetry.update();
            } else {
                telemetry.addData("BST Error", solution.message);
                telemetry.update();
            }
        } else {
            throw new IllegalArgumentException("No goal position found for target ID");
        }
    }

    private void checkSpeedAndLaunch(boolean shouldAim, int targetTagId) {
        if (!waitingForSpeed) return;

        // 等待期间每帧重新计算瞄准角度和弹道，补偿机器人运动
        //潜在误差：speed没有实时更新，但否则pid可能收敛不了，以后再想办法
        if (shouldAim && targetTagId >= 0) {
            Object[] aimResult = aim(targetTagId);
            double aimRoll = (double) aimResult[1];
            double aimYaw = (double) aimResult[2];
            double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
            double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(aimRoll));
            double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(aimRoll));
            BSTsolver.Solution solution = bstSolver.predict(
                RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(),
                targetX, targetY);
            if (solution.success) {
                turretDegreeController.rotateTo(Math.toDegrees(solution.roll), solution.yaw);
            }
        } else if (targetTagId >= 0) {
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
                    turretDegreeController.rotateTo(Math.toDegrees(solution.roll), solution.yaw);
                }
            }
        }

        if (shooter.reachedVelocity() && turretDegreeController.reachedTarget()) {
            launch();
            waitingForSpeed = false;
        } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
            waitingForSpeed = false;
            telemetry.addData("Shooter", "Speed wait timeout");
            telemetry.update();
        }
    }

    public void launch() {
        //todo: 打开闸门
        lastLaunchTime = System.currentTimeMillis();
    }

    public void reset(){
        //todo: 关闭闸门
        shooting = false;
        waitingForSpeed = false;
    }

    public void update(boolean shouldAim, boolean shouldShoot,int targetTagId) {
        shooter.update();
        turretDegreeController.update();

        if (shooting && !waitingForSpeed && System.currentTimeMillis() - lastLaunchTime > LAUNCH_COOLDOWN_MS) {
            shooting = false;
        }

        if (waitingForSpeed) {
            checkSpeedAndLaunch(shouldAim, targetTagId);
            return;
        }

        if (shouldAim && !shooting) {
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
                shootByPosition(targetTagId);
                telemetry.addData("Warning", "Response unavailable, auto downgrading");
                telemetry.update();
            }
        } else if (shouldShoot && !shooting) {
            shootByPosition(targetTagId);
        } else if (!shouldShoot) {
            reset();
        }
    }
    //重载输入统一
    public void update(int speed, boolean shouldShoot) {
        shooter.update();
        turretDegreeController.update();

        if (waitingForSpeed) {
            if (shooter.reachedVelocity()) {
                launch();
                waitingForSpeed = false;
            } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
                waitingForSpeed = false;
            }
            return;
        }

        if (System.currentTimeMillis() - lastLaunchTime < LAUNCH_COOLDOWN_MS) {
            return;
        }

        if (shouldShoot) {
            shooter.setTargetSpeed(speed);
            waitingForSpeed = true;
            speedWaitStartTime = System.currentTimeMillis();
            telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
            telemetry.update();
        } else {
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

        if (waitingForSpeed) {
            if (shooter.reachedVelocity() && turretDegreeController.reachedTarget()) {
                launch();
                waitingForSpeed = false;
            } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
                waitingForSpeed = false;
            }
            return;
        }

        if (System.currentTimeMillis() - lastLaunchTime < LAUNCH_COOLDOWN_MS) {
            return;
        }

        if (shouldShoot) {
            shooter.setTargetSpeed(speed);
            waitingForSpeed = true;
            speedWaitStartTime = System.currentTimeMillis();
            telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
            telemetry.update();
        } else {
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
