package org.firstinspires.ftc.teamcode.Controllers.Turret;

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

import java.util.List;

public class Turret {
    public Shooter shooter;
    Telemetry telemetry;
    public TurretDegreeController turretDegreeController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double roll;
    private double yaw;
    private BSTsolver bstSolver;

    private boolean waitingForSpeed = false;
    private long speedWaitStartTime = 0;
    private long lastLaunchTime = 0;
    private static final long SPEED_WAIT_TIMEOUT_MS = 500;
    private static final long LAUNCH_COOLDOWN_MS = 300;

    public enum ShootPhase { IDLE, PREPARING, ACCELERATING, FIRING }
 
     public ShootPhase getShootPhase() {
         return shootPhase;
     }
    private ShootPhase shootPhase = ShootPhase.IDLE;
    private boolean useVisionForAiming = true;
    private int currentTargetTagId;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        roll = 0.0;
        yaw = 0.0;

        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry = telemetry;

        turretDegreeController = new TurretDegreeController(hardwareMap, telemetry);
        bstSolver = new BSTsolver();

        initAprilTag(hardwareMap);
    }

    private void initAprilTag(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .build();

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
            this.roll = ((roll % 360) + 540) % 360 - 180;
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

    /**
     * 统一瞄准接口
     * @param allowVision true=优先视觉(检测不到自动降级为定位), false=纯定位
     * @return Object[]{isTargetFound, targetRoll, targetYaw-WebcamTheta}
     */
    public Object[] aim(boolean allowVision) {
        get_angle();

        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean isTargetFound = false;
        AprilTagDetection targetDetection = null;

        if (allowVision) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == currentTargetTagId) {
                    targetDetection = detection;
                    isTargetFound = true;
                    break;
                }
            }
        }

        double targetRoll = 0;
        double targetYaw = 0;

        if (isTargetFound && targetDetection != null) {
            double bearing = targetDetection.ftcPose.bearing;
            double elevation = targetDetection.ftcPose.elevation;
            targetRoll = this.roll + bearing;
            targetYaw = this.yaw + elevation;
            if (targetRoll <= -180 || targetRoll >= 180) {
                isTargetFound = false;
            }
        }

        if (!isTargetFound) {
            double[] goalPos = HypParams.getGoalPosition(currentTargetTagId);
            if (goalPos != null) {
                double robotX = RobotPosition.getInstance().getX();
                double robotY = RobotPosition.getInstance().getY();
                double robotTheta = RobotPosition.getInstance().getTheta();

                double dx = goalPos[0] - robotX;
                double dy = goalPos[1] - robotY;

                double relativeDx = dx * Math.cos(robotTheta) + dy * Math.sin(robotTheta);
                double relativeDy = -dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);

                double theta = Math.atan2(relativeDy, relativeDx);
                targetRoll = Math.toDegrees(theta);
                targetYaw = Math.toDegrees(Math.atan2(HypParams.TagH, Math.hypot(relativeDx, relativeDy))) + HypParams.WebcamTheta;
            } else {
                throw new IllegalArgumentException("Goal position not found for targetTagId: " + currentTargetTagId);
            }
        }

        rotate_to(targetRoll, targetYaw);
        return new Object[]{isTargetFound, targetRoll, targetYaw - HypParams.WebcamTheta};
    }

    /**
     * 统一射击管理接口（状态机驱动）
     * IDLE → PREPARING(等待+sweeper反转) → ACCELERATING(飞轮加速) → FIRING(持续发射) → IDLE(shouldShoot==false)
     * @param allowVision true=视觉瞄准, false=定位瞄准
     */
    public void shoot(boolean allowVision) {
        switch (shootPhase) {
            case IDLE: {
                useVisionForAiming = allowVision;
                shootPhase = ShootPhase.PREPARING;
                speedWaitStartTime = System.currentTimeMillis();
                telemetry.addData("ShootPhase", "PREPARING");
                telemetry.update();
                break;
            }

            case PREPARING: {
                aim(useVisionForAiming);
                if (System.currentTimeMillis() - speedWaitStartTime >= HypParams.WaitTime) {
                    BSTsolver.Solution solution = solveBST();
                    if (solution.success) {
                        shootPhase = ShootPhase.ACCELERATING;
                        shooter.setTargetVelocity(solution.speed);
                        waitingForSpeed = true;
                        speedWaitStartTime = System.currentTimeMillis();
                        telemetry.addData("BST Init", "speed=%d", solution.speed);
                        telemetry.addData("ShootPhase", "ACCELERATING");
                        telemetry.update();
                    } else {
                        telemetry.addData("BST Error", solution.message);
                        shootPhase = ShootPhase.IDLE;
                        telemetry.update();
                    }
                }
                break;
            }

            case ACCELERATING: {
                aim(useVisionForAiming);
                if (shooter.reachedVelocity()) {
                    shootPhase = ShootPhase.FIRING;
                    speedWaitStartTime = System.currentTimeMillis();
                    BSTsolver.Solution solution = solveBST();
                    if (solution.success) {
                        turretDegreeController.rotateTo(Math.toDegrees(solution.roll), solution.yaw);
                        telemetry.addData("BST Final", "roll=%.2f yaw=%d", Math.toDegrees(solution.roll), solution.yaw);
                    }
                    telemetry.addData("Shooter", "Speed reached, start firing");
                    telemetry.addData("ShootPhase", "FIRING");
                    telemetry.update();
                } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
                    shootPhase = ShootPhase.IDLE;
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Speed wait timeout");
                    telemetry.update();
                }
                break;
            }

            case FIRING: {
                aim(useVisionForAiming);
                BSTsolver.Solution solution = solveBST();
                if (solution.success) {
                    turretDegreeController.rotateTo(Math.toDegrees(solution.roll), solution.yaw);
                }
                // 飞轮速度由 shooter.update() 持续维持
                // sweeper正转由上层代码根据 ShootPhase.FIRING 持续驱动
                break;
            }
        }
    }

    private BSTsolver.Solution solveBST() {
        if (useVisionForAiming) {
            Object[] aimResult = aim(true);
            double aimRoll = (double) aimResult[1];
            double aimYaw = (double) aimResult[2];
            double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
            double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(aimRoll));
            double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(aimRoll));
            return bstSolver.predict(
                RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(),
                targetX, targetY);
        } else {
            double[] goalPos = HypParams.getGoalPosition(currentTargetTagId);
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
                return bstSolver.predict(vx, vy, relativeDx, relativeDy);
            }
            return new BSTsolver.Solution("No goal position for tag " + currentTargetTagId);
        }
    }

    public void launch() {
        waitingForSpeed = false;
        shootPhase = ShootPhase.IDLE;
        lastLaunchTime = System.currentTimeMillis();
    }

    public void reset(){
        waitingForSpeed = false;
        shooter.setTargetVelocity(0);
        shootPhase = ShootPhase.IDLE;
    }

    public void update(boolean AllowVision, boolean shouldShoot, int targetTagId) {
        this.currentTargetTagId = targetTagId;
        shooter.update();
        turretDegreeController.update();

        if (shootPhase != ShootPhase.IDLE) {
            if (!shouldShoot) {
                reset();
                return;
            }
            shoot(AllowVision);
            return;
        }

        if (shouldShoot) {
            shoot(AllowVision);
        } else {
            reset();
            aim(AllowVision);
        }
    }

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
            shooter.setTargetVelocity(speed);
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
            shooter.setTargetVelocity(speed);
            waitingForSpeed = true;
            speedWaitStartTime = System.currentTimeMillis();
            telemetry.addData("Shooting", "yaw= %.2f, roll= %.2f, speed= %d", yaw, roll, speed);
            telemetry.update();
        } else {
            reset();
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