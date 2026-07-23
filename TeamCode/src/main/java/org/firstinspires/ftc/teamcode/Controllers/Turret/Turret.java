package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.BST.BSTsolver;

import java.util.List;
@Config
public class Turret {
    public Shooter shooter;
    Telemetry telemetry;
    public TurretDegreeController turretDegreeController;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double roll;
    private double yaw;
    private BSTsolver bstSolver;

    private Servo triggerServo;

    private boolean shooting = false;
    private boolean waitingForSpeed = false;
    private long speedWaitStartTime = 0;
    private long lastLaunchTime = 0;
    private static final long SPEED_WAIT_TIMEOUT_MS = 2000;
    private static final long LAUNCH_COOLDOWN_MS = 300;
    private static final long AIM_TIMEOUT_MS = 2000;

    private enum ShootPhase { IDLE, ACCELERATING, AIMING }
    private ShootPhase shootPhase = ShootPhase.IDLE;
    private boolean useVisionForAiming = true;
    private double finalAimRoll;
    private double finalAimYaw;
    private int currentTargetTagId;
    private boolean isLaunching = false;
    
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private int preSpeed = 0;

    // 反转模式（P1 右 Bumper 触发）：飞轮反转 + 扳机舵机到发射位置，用于排球
    private boolean reverseMode = false;

    // 最近一次 aim() 的结果跟踪（用于 Telemetry 显示）
    private boolean lastAimIsTargetFound = false;
    private double lastAimTargetRoll = 0;
    private double lastAimTargetYaw = 0;
    private int lastAimDetectionCount = 0;

    // 视觉滞后滤波：防止检测闪烁导致角度跳变
    public static int VISION_DROP_THRESHOLD = 30; // 允许连续丢失的最大帧数
    private int visionDropFrames = VISION_DROP_THRESHOLD; // 当前连续丢失帧数（初始为阈值，表示未获得过视觉数据）
    private double lastVisionRoll = 0; // 最后一帧有效视觉的 roll
    private double lastVisionYaw = 0; // 最后一帧有效视觉的 yaw
    private int lastTargetTagId = -1; // 上一帧的目标标签 ID，用于检测切换

    /**
     * 设置反转模式
     * 反转模式下飞轮反转，扳机舵机移到发射位置，用于从进料口反向排球
     */
    public void setReverse(boolean reverse) {
        this.reverseMode = reverse;
    }

    /**
     * 设置手柄引用，用于发射失败时震动反馈
     */
    public void setGamepads(Gamepad g1, Gamepad g2) {
        this.gamepad1 = g1;
        this.gamepad2 = g2;
    }

    /**
     * 发射失败时震动所有手柄 + telemetry提示
     */
    private void rumbleOnFail(String reason) {
        telemetry.addData("Shooting FAIL", reason);
        telemetry.update();
        if (gamepad1 != null) gamepad1.rumbleBlips(3);
        if (gamepad2 != null) gamepad2.rumbleBlips(3);
    }

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        roll = 0.0;
        yaw = 0.0;

        shooter = new Shooter(hardwareMap, telemetry);
        this.telemetry = telemetry;

        turretDegreeController = new TurretDegreeController(hardwareMap, telemetry);
        bstSolver = new BSTsolver();

        triggerServo = hardwareMap.get(Servo.class, "trigger");
        triggerServo.setPosition(HypParams.triggerResetPosition);

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

        // 如果目标标签切换，立即重置视觉滤波状态
        if (currentTargetTagId != lastTargetTagId) {
            visionDropFrames = VISION_DROP_THRESHOLD;
            lastTargetTagId = currentTargetTagId;
        }

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
            // 视觉检测成功：重置丢失计数，保存视觉数据
            visionDropFrames = 0;
            //webcam顺时针为正，程序内统一以逆时针为正
            double bearing = -targetDetection.ftcPose.bearing;
            double elevation = targetDetection.ftcPose.elevation;
            targetRoll = this.roll - bearing;
            targetYaw = HypParams.WebcamTheta + elevation;
            lastVisionRoll = targetRoll;
            lastVisionYaw = targetYaw;
            // 即使超出射界，仍保留视觉数据更新 targetRoll 以保持跟踪
            // 物理限幅由 TurretDegreeController.setTargetRoll() 处理
        } else if (allowVision && visionDropFrames < VISION_DROP_THRESHOLD) {
            // 视觉短暂丢失，未超过阈值：继续使用上一帧有效视觉数据，避免角度跳变
            visionDropFrames++;
            targetRoll = lastVisionRoll;
            targetYaw = lastVisionYaw;
            isTargetFound = true; // 标记为视觉有效（滤波后）
        } else {
            // 视觉丢失超过阈值或未启用视觉：使用定位计算
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

        // 更新 telemetry 跟踪数据
        lastAimIsTargetFound = isTargetFound;
        lastAimTargetRoll = targetRoll;
        lastAimTargetYaw = targetYaw;
        lastAimDetectionCount = detections.size();

        rotate_to(targetRoll, HypParams.BestYaw);
        return new Object[]{isTargetFound, targetRoll, targetYaw};
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
                // 先尝试 aim，获取视觉是否成功的信息
                Object[] aimResult = aim(allowVision);
                boolean visionOk = allowVision && (boolean) aimResult[0];

                if (visionOk) {
                    // 视觉有效：使用视觉测距公式
                    double aimRoll = (double) aimResult[1];
                    double aimYaw = (double) aimResult[2];
                    double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
                    double d = HypParams.TagH * cotYaw + HypParams.WebCamCenterDistance;
                    double targetX = d * Math.cos(Math.toRadians(aimRoll));
                    double targetY = d * Math.sin(Math.toRadians(aimRoll));
                    solution = bstSolver.predict(
                        RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(),
                        targetX, targetY);
                } else {
                    // 视觉失效或未启用：使用定位计算
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
                        rumbleOnFail("No goal position for tag " + currentTargetTagId);
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
                    rumbleOnFail("BST solve failed: " + solution.message);
                }
                break;
            }

            case ACCELERATING: {
                Object[] aimResult = aim(useVisionForAiming);
                if (shooter.reachedVelocity()) {
                    shootPhase = ShootPhase.AIMING;
                    speedWaitStartTime = System.currentTimeMillis();

                    BSTsolver.Solution solution;
                    boolean visionOk = useVisionForAiming && (boolean) aimResult[0];
                    if (visionOk) {
                        // 视觉有效：使用视觉测距公式
                        double aimRoll = (double) aimResult[1];
                        double aimYaw = (double) aimResult[2];
                        double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
                        double dWebcam = HypParams.TagH * cotYaw;
                        double targetX = dWebcam * Math.cos(Math.toRadians(aimRoll)) + HypParams.WebCamCenterDistance;
                        double targetY = dWebcam * Math.sin(Math.toRadians(aimRoll));
                        solution = bstSolver.predict(
                            RobotPosition.getInstance().getVx(), RobotPosition.getInstance().getVy(),
                            targetX, targetY);
                    } else {
                        // 视觉失效或未启用：使用定位计算
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
                        
                        // 若计算出的平转角度超出硬件允许范围，中止发射
                        if (Math.abs(finalAimRoll) > HypParams.maxRoll) {
                            rumbleOnFail(String.format("Roll out of range (%.1f°), abort", finalAimRoll));
                            shootPhase = ShootPhase.IDLE;
                            shooting = false;
                            waitingForSpeed = false;
                            break;
                        }
                        
                        turretDegreeController.rotateTo(finalAimRoll, finalAimYaw);
                        telemetry.addData("BST Final", "roll=%.2f yaw=%.2f", finalAimRoll, finalAimYaw);
                    }
                    telemetry.addData("Shooter", "Speed reached, turret aiming");
                    telemetry.update();
                } else if (System.currentTimeMillis() - speedWaitStartTime > SPEED_WAIT_TIMEOUT_MS) {
                    rumbleOnFail("Speed wait timeout");
                    shootPhase = ShootPhase.IDLE;
                    shooting = false;
                    waitingForSpeed = false;
                }
                break;
            }

            case AIMING: {
                if (turretDegreeController.reachedTarget()) {
                    launch();
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Launching");
                    telemetry.update();
                } else if (System.currentTimeMillis() - speedWaitStartTime > AIM_TIMEOUT_MS) {
                    launch();
                    waitingForSpeed = false;
                    telemetry.addData("Shooter", "Aim timeout, force launch");
                    telemetry.update();
                }
                break;
            }
        }
    }

    public void launch() {
        triggerServo.setPosition(HypParams.triggerLaunchPosition);
        isLaunching = true;
        lastLaunchTime = System.currentTimeMillis();
    }

    public boolean isLaunching() {
        return isLaunching;
    }

    /**
     * 强制发射：不论是否达速，立即释放扳机并标记为发射中
     * 用于二操右扳机强制送球场景
     */
    public void forceLaunch() {
        triggerServo.setPosition(HypParams.triggerLaunchPosition);
        isLaunching = true;
        lastLaunchTime = System.currentTimeMillis();
        shooting = true;
        waitingForSpeed = false;
        shootPhase = ShootPhase.IDLE;
    }

    // ======== Telemetry 查询接口 ========
    public boolean isLastAimTargetFound() { return lastAimIsTargetFound; }
    public double getLastAimTargetRoll() { return lastAimTargetRoll; }
    public double getLastAimTargetYaw() { return lastAimTargetYaw; }
    public int getLastAimDetectionCount() { return lastAimDetectionCount; }
    public int getVisionDropFrames() { return visionDropFrames; }

    /** 获取炮台当前状态字符串 */
    public String getState() {
        if (reverseMode) return "REVERSE";
        if (shootPhase != ShootPhase.IDLE) return shootPhase.toString();
        if (isLaunching) return "LAUNCHING";
        if (shooting) return "COOLDOWN";
        return "IDLE";
    }

    public void reset(){
        triggerServo.setPosition(HypParams.triggerResetPosition);
        shooting = false;
        waitingForSpeed = false;
        isLaunching = false;
        shooter.setTargetVelocity(preSpeed);
        shootPhase = ShootPhase.IDLE;
    }

    public void update(boolean AllowVision, boolean shouldShoot, int targetTagId) {
        update(AllowVision, shouldShoot, targetTagId, 0);
    }

    public void update(boolean AllowVision, boolean shouldShoot, int targetTagId, int preSpeed) {
        this.preSpeed = preSpeed;
        this.currentTargetTagId = targetTagId;

        // 反转模式：飞轮反转 + 扳机舵机到发射位置，用于从进料口反向排球
        if (reverseMode) {
            shooter.setTargetVelocity(HypParams.shooterReverseSpeed);
            triggerServo.setPosition(HypParams.triggerLaunchPosition);
            shooter.update();
            turretDegreeController.update();
            return;
        }

        shooter.update();
        turretDegreeController.update();

        if (shootPhase != ShootPhase.IDLE) {
            if (!shouldShoot) {
                reset();
                aim(AllowVision);
                return;
            }
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
        update(speed, shouldShoot, 0);
    }

    public void update(int speed, boolean shouldShoot, int preSpeed) {
        this.preSpeed = preSpeed;
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
        update(roll, yaw, kx, 0);
    }

    public void update(double roll, double yaw, double kx, int preSpeed) {
        this.preSpeed = preSpeed;
        shooter.update();
        shooter.setTargetVelocity(this.preSpeed);
        DcMotorEx rollMotor = turretDegreeController.rollMotor;
        rollMotor.setPower(kx);
    }

    public void update(double roll, double yaw, int speed, boolean shouldShoot) {
        update(roll, yaw, speed, shouldShoot, 0);
    }

    public void update(double roll, double yaw, int speed, boolean shouldShoot, int preSpeed) {
        this.preSpeed = preSpeed;

        // 反转模式：飞轮反转 + 扳机舵机到发射位置，用于从进料口反向排球
        if (reverseMode) {
            shooter.setTargetVelocity(HypParams.shooterReverseSpeed);
            triggerServo.setPosition(HypParams.triggerLaunchPosition);
            shooter.update();
            return;
        }

        shooter.update();
        turretDegreeController.rotateTo(roll, yaw);
        turretDegreeController.update();

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