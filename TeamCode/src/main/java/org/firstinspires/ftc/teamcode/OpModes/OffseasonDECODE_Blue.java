package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@TeleOp(name = "OffseasonDECODE_Blue", group = "A_OffseasonDECODE")
public class OffseasonDECODE_Blue extends LinearOpMode {
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    private double roll = 0.0;
    private double yaw = 50.0;
    private int targetSpeed = 0;
    private boolean isShooting = false;

    private enum AIM_MODE { VISION, LOCALIZATION, MANUAL }
    private AIM_MODE aimMode = AIM_MODE.MANUAL;

    private int targetTagId = 24;

    private int preSpeed = 0;

    private TeamColor teamColor = TeamColor.BLUE;

    private long lastFrameTime = 0;
    private double fps = 0;
    private double frameTimeMs = 0;

    // 各模块耗时统计（微秒）
    private double timeRobotPosUs = 0;
    private double timeChassisUs = 0;
    private double timeTurretUs = 0;
    private double timeSweeperUs = 0;
    private double timeTelemetryUs = 0;

    private static final double ROLL_SPEED = 2.0;
    private static final double YAW_STEP = 5.0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.setGamepads(gamepad1, gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team Color", "BLUE");
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("B", "Reset Pose to Blue ResetPose");
        telemetry.addData("A", "Emergency Stop");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("Y", "Sweeper Stop");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.addData("X", "Cycle Aim Mode: VISION/LOCALIZATION/MANUAL");
        telemetry.addData("Left Stick X", "Turret Roll (MANUAL mode only)");
        telemetry.addData("D-Pad Up/Down", "Yaw +/-5 (MANUAL mode only)");
        telemetry.addData("D-Pad Left/Right", "Speed +/-100 (MANUAL mode only)");
        telemetry.addData("Y", "Toggle Shoot"); 
        telemetry.addData("Right Trigger", "Preload Speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 帧率计算（原始帧间隔，不加平滑）
            long now = System.nanoTime();
            if (lastFrameTime > 0) {
                frameTimeMs = (now - lastFrameTime) / 1_000_000.0;
                fps = 1000.0 / frameTimeMs;
            }
            lastFrameTime = now;

            long t0 = System.nanoTime();
            RobotPosition.getInstance().update();
            timeRobotPosUs = (System.nanoTime() - t0) / 1000.0;

            // ======== P1 Controls ========

            t0 = System.nanoTime();
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            timeChassisUs = (System.nanoTime() - t0) / 1000.0;

            if (gamepad1.xWasPressed()) {
                chassis.exchangeUseNoHeadMode();
            }

            // 重置定位到对应颜色的 ResetPose
            if (gamepad1.bWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
                telemetry.addData("ResetPose", "Reset to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red"));
            }

            if (gamepad1.aWasPressed()) {
                chassis.stop();
                turret.stop();
                sweeper.setStop();
                sweeper.update();
            }

            if (isShooting) {
                sweeper.setGiveArtifact();
            } else {
                // 球满时自动停止，除非操作手按住左右 bumper 强行继续
                boolean driverOverride = gamepad1.left_bumper || gamepad1.right_bumper;
                if (RobotPosition.getInstance().isFull() && !driverOverride) {
                    sweeper.setStop();
                } else if (gamepad1.left_bumper) {
                    sweeper.setEat();
                } else if (gamepad1.right_bumper) {
                    sweeper.setOutput();
                } else if (gamepad1.a) {
                    sweeper.setStop();
                }
            }

            // 右 bumper 释放时恢复炮台正常模式（总是在 sweeper 控制之后执行）
            if (!gamepad1.right_bumper) {
                turret.setReverse(false);
            }

            // ======== P2 Controls ========

            // 循环切换瞄准模式：VISION → LOCALIZATION → MANUAL → VISION
            if (gamepad2.xWasPressed()) {
                switch (aimMode) {
                    case VISION:    aimMode = AIM_MODE.LOCALIZATION; break;
                    case LOCALIZATION: aimMode = AIM_MODE.MANUAL; break;
                    case MANUAL:    aimMode = AIM_MODE.VISION; break;
                }
                telemetry.addData("Aim Mode", aimMode);
            }

            if (gamepad2.aWasPressed()) {
                chassis.stop();
                turret.stop();
                sweeper.setStop();
                sweeper.update();
            }

            // --- MANUAL 模式：手动控制 Roll / Yaw / Speed ---
            if (aimMode == AIM_MODE.MANUAL) {
                roll += gamepad2.left_stick_x * ROLL_SPEED;

                if (gamepad2.dpad_up) {
                    yaw += YAW_STEP;
                }
                if (gamepad2.dpad_down) {
                    yaw -= YAW_STEP;
                }

                if (gamepad2.dpad_right) {
                    targetSpeed += SPEED_STEP;
                }
                if (gamepad2.dpad_left) {
                    targetSpeed -= SPEED_STEP;
                }
                targetSpeed = Math.max(SPEED_MIN, Math.min(SPEED_MAX, targetSpeed));

                // 发射开关
                isShooting = gamepad2.y;

                // 预载飞轮速度（P2 右扳机）
                preSpeed = Math.round(gamepad2.right_trigger * HypParams.maxPreSpeed);

                t0 = System.nanoTime();
                turret.update(roll, yaw, targetSpeed, isShooting, preSpeed);
                timeTurretUs = (System.nanoTime() - t0) / 1000.0;

            } else {
                // --- VISION / LOCALIZATION 模式：自动瞄准 ---
                boolean allowVision = (aimMode == AIM_MODE.VISION);

                // 预载飞轮速度（P2 右扳机）
                preSpeed = Math.round(gamepad2.right_trigger * HypParams.maxPreSpeed);

                // 发射开关
                isShooting = gamepad2.y;

                t0 = System.nanoTime();
                turret.update(allowVision, isShooting, targetTagId, preSpeed);
                timeTurretUs = (System.nanoTime() - t0) / 1000.0;
            }

            // ======== Update & Telemetry ========

            t0 = System.nanoTime();
            sweeper.update();
            timeSweeperUs = (System.nanoTime() - t0) / 1000.0;

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());

            // 瞄准模式显示（含视觉降级指示）
            String aimModeDisplay;
            if (aimMode == OffseasonDECODE_Blue.AIM_MODE.VISION) {
                aimModeDisplay = turret.isLastAimTargetFound() ? "VISION" : "VISION(LOCALIZATION)";
            } else {
                aimModeDisplay = aimMode.toString();
            }
            telemetry.addData("Aim Mode", aimModeDisplay);
            telemetry.addData("PreSpeed", "%d RPM", preSpeed);
            telemetry.addData("Current Speed", "%.0f RPM", turret.shooter.getCurrentVelocity());
            telemetry.addData("Roll", "%.2f deg", roll);
            telemetry.addData("Yaw", "%.2f deg", yaw);
            telemetry.addData("Target Speed", "%d RPM", targetSpeed);
            telemetry.addData("Shooting", isShooting ? "ACTIVE" : "IDLE");
            telemetry.addData("Reverse Mode", gamepad1.right_bumper ? "ACTIVE" : "OFF");

            // 位姿信息
            telemetry.addData("Pose X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Pose Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Pose Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            // 目标信息
            telemetry.addData("Target Tag ID", targetTagId);
            double[] goalPos = HypParams.getGoalPosition(targetTagId);
            if (goalPos != null) {
                telemetry.addData("Target Coords", "(%.1f, %.1f)", goalPos[0], goalPos[1]);
            }
            telemetry.addData("Target Roll", "%.2f deg", turret.getLastAimTargetRoll());

            // Webcam 检测结果
            telemetry.addData("Webcam Detections", turret.getLastAimDetectionCount());
            telemetry.addData("Webcam Tag Found", turret.isLastAimTargetFound() ? "YES" : "NO");
            telemetry.addData("Vision Drop Frames", turret.getVisionDropFrames() + "/" + 10);
            if (turret.isLastAimTargetFound()) {
                telemetry.addData("Webcam Target Yaw", "%.2f deg", turret.getLastAimTargetYaw());
            }

            chassis.telemetry();
            sweeper.setTelemetry();
            // === 帧率 & 各模块耗时（微秒） ===
            telemetry.addData("FPS / Frame", "%.1f / %.1f ms", fps, frameTimeMs);
            telemetry.addData("Time: RobotPos", "%.0f us", timeRobotPosUs);
            telemetry.addData("Time: Chassis", "%.0f us", timeChassisUs);
            telemetry.addData("Time: Turret", "%.0f us", timeTurretUs);
            telemetry.addData("Time: Sweeper", "%.0f us", timeSweeperUs);
            // === 各硬件 setPower 耗时（微秒） ===
            long chassisSP = RobotPosition.getInstance().getDrive().chassisSetPowerNs;
            long shooterSP = turret.shooter.shooterSetPowerNs;
            long turretSP = turret.turretDegreeController.turretSetPowerNs;
            long sweeperSP = sweeper.sweeperSetPowerNs;
            telemetry.addData("setPower: Chassis", "%.1f us", chassisSP / 1000.0);
            telemetry.addData("setPower: Shooter", "%.1f us", shooterSP / 1000.0);
            telemetry.addData("setPower: Turret", "%.1f us", turretSP / 1000.0);
            telemetry.addData("setPower: Sweeper", "%.1f us", sweeperSP / 1000.0);
            // 重置本帧 setPower 计时
            RobotPosition.getInstance().getDrive().chassisSetPowerNs = 0;
            turret.shooter.shooterSetPowerNs = 0;
            turret.turretDegreeController.turretSetPowerNs = 0;
            sweeper.sweeperSetPowerNs = 0;
            // telemetry.update() 耗时
            telemetry.addData("Time: Telemetry", "%.0f us", timeTelemetryUs);
            long tTel = System.nanoTime();
            telemetry.update();
            timeTelemetryUs = (System.nanoTime() - tTel) / 1000.0;
        }

        chassis.stop();
        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}