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

    private boolean shooting = false;
    private boolean waitingForSpeed = false;
    private long speedWaitStartTime = 0;
    private long lastLaunchTime = 0;
    private static final long SPEED_WAIT_TIMEOUT_MS = 500;
    private static final long LAUNCH_COOLDOWN_MS = 300;
    private static final long AIM_TIMEOUT_MS = 2000;

    private enum ShootPhase { IDLE, ACCELERATING, AIMING }
    private ShootPhase shootPhase = ShootPhase.IDLE;
    private boolean useVisionForAiming = true;
    private double finalAimRoll;
    private double finalAimYaw;
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
     * IDLE → 第一次BST解算初速度 → ACCELERATING → 达速后第二次BST解算roll/yaw → AIMING → 旋转到位 → launch
     * @param allowVision true=视觉瞄准, false=定位瞄准
     */
    public void shoot(boolean allowVision) {
        switch (shootPhase) {
            case IDLE: {
                useVisionForAiming = allowVision;
                BSTsolver.Solution solution;
                if (allowVision) {
                    Object[] aimResult = aim(true);
                    double aimRoll = (double) aimResult[1];
                    double aimYaw = (double) aimResult[2];
                    double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
                    double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(aimRoll));
                    double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(aimRoll));
                    solution = bstSolver.predict(
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
                        solution = bstSolver.predict(vx, vy, relativeDx, relativeDy);
                    } else {
                        telemetry.addData("BST Error", "No goal position for tag %d", currentTargetTagId);
                        telemetry.update();
                        break;
                    }
                }
                if (solution.success) {
                    shooting = true;
                    shootPhase = ShootPhase.ACCELERATING;
                    shooter.setTargetVelocity(solution.speed);
                    waitingForSpeed = true;
                    speedWaitStartTime = System.currentTimeMillis();
                    telemetry.addData("BST Init", "speed=%d", solution.speed);
                    telemetry.update();
                } else {
                    telemetry.addData("BST Error", solution.message);
                    telemetry.update();
                }
                break;
            }

            case ACCELERATING: {
                aim(useVisionForAiming);
                if (shooter.reachedVelocity()) {
                    shootPhase = ShootPhase.AIMING;
                    speedWaitStartTime = System.currentTimeMillis();

                    BSTsolver.Solution solution;
                    if (useVisionForAiming) {
                        Object[] aimResult = aim(true);
                        double aimRoll = (double) aimResult[1];
                        double aimYaw = (double) aimResult[2];
                        double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
                        double targetX = HypParams.TagH * cotYaw * Math.cos(Math.toRadians(aimRoll));
                        double targetY = HypParams.TagH * cotYaw * Math.sin(Math.toRadians(aimRoll));
                        solution = bstSolver.predict(
                            RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(),
                            targetX, targetY);
                    } else {
                        double[] goalPos = HypParams.getGoalPosition(currentTargetTagId);
                        double robotX = RobotPosition.getInstance().getX();
                        double robotY = RobotPosition.getInstance().getY();
                        double robotTheta = RobotPosition.getInstance().getTheta();
                        double vx = RobotPosition.getInstance().getVx();
                        double vy = RobotPosition.getInstance().getVy();
                        double dx = goalPos[0] - robotX;
                        double dy = goalPos[1] - robotY;
                        double relativeDx = dx * Math.cos(robotTheta) + dy * Math.sin(robotTheta);
                        double relativeDy = -dx * Math.sin(robotTheta) + dy * Math.cos(robotTheta);
                        solution = bstSolver.predict(vx, vy, relativeDx, relativeDy);
                    }
                    if (solution.success) {
                        finalAimRoll = Math.toDegrees(solution.roll);
                        finalAimYaw = solution.yaw;
                        turretDegreeController.rotateTo(finalAimRoll, finalAimYaw);
                        telemetry.addData("BST Final", "roll=%.2f yaw=%.2f", finalAimRoll, finalAimYaw);
                    }
                    telemetry.addData("Shooter", "Speed reached, turret aiming");
                    telemetry.update();
                } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
                    shootPhase = ShootPhase.IDLE;
                    shooting = false;
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Speed wait timeout");
                    telemetry.update();
                }
                break;
            }

            case AIMING: {
                if (turretDegreeController.reachedTarget()) {
                    launch();
                    shootPhase = ShootPhase.IDLE;
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Launched");
                    telemetry.update();
                } else if (System.currentTimeMillis() - speedWaitStartTime > AIM_TIMEOUT_MS) {
                    launch();
                    shootPhase = ShootPhase.IDLE;
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Aim timeout, force launch");
                    telemetry.update();
                }
                break;
            }
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
        shooter.setTargetVelocity(0);
        shootPhase = ShootPhase.IDLE;
    }

    public void update(boolean AllowVision, boolean shouldShoot, int targetTagId) {
        this.currentTargetTagId = targetTagId;
        shooter.update();
        turretDegreeController.update();

        if (shootPhase != ShootPhase.IDLE) {
            shoot(useVisionForAiming);
            return;
        }

        if (shooting && !waitingForSpeed && System.currentTimeMillis() - lastLaunchTime > LAUNCH_COOLDOWN_MS) {
            shooting = false;
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