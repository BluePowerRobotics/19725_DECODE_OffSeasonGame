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

@TeleOp(name = "OffseasonDECODE_Red", group = "A_OffseasonDECODE")
public class OffseasonDECODE_Red extends LinearOpMode {
private Chassis chassis;
private Sweeper sweeper;
private Turret turret;
private ActionRunner actionRunner;

private double roll = 0.0;
private double yaw = 50.0;
private int targetSpeed = 0;
private boolean isShooting = false;

private enum AIM_MODE { VISION, LOCALIZATION, MANUAL }
private AIM_MODE aimMode = AIM_MODE.VISION;

private int targetTagId = 20;

private int preSpeed = 0;

private TeamColor teamColor = TeamColor.RED;

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
    telemetry.addData("Team Color", "RED");
    telemetry.addData("--- P1 Controls ---", "");
    telemetry.addData("Left Stick", "Chassis Drive");
    telemetry.addData("Right Stick X", "Chassis Rotation");
    telemetry.addData("X", "Toggle No-Head Mode");
    telemetry.addData("B", "Reset Pose to Red ResetPose");
    telemetry.addData("A", "Emergency Stop");
    telemetry.addData("Left Bumper", "Sweeper Eat");
    telemetry.addData("Right Bumper", "Sweeper Output");
    telemetry.addData("Y", "Sweeper Stop");
    telemetry.addData("--- P2 Controls ---", "");
    telemetry.addData("X", "Cycle Aim Mode: VISION/LOCALIZATION/MANUAL");
    telemetry.addData("Left Stick X", "Turret Roll (MANUAL mode only)");
    telemetry.addData("D-Pad Up/Down", "Yaw +/-5 (MANUAL mode only)");
    telemetry.addData("D-Pad Left/Right", "Speed +/-100 (MANUAL mode only)");
    telemetry.addData("Y", "(Hold) Shoot");
    telemetry.addData("A", "Auto Sweeper by Turret");
    telemetry.addData("Right Trigger", "Preload Speed");
    telemetry.update();

    waitForStart();

    while (opModeIsActive()) {
        RobotPosition.getInstance().update();

        // ======== P1 Controls ========

        chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
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
                case VISION:
                    aimMode = AIM_MODE.LOCALIZATION;
                    break;
                case LOCALIZATION:
                    aimMode = AIM_MODE.MANUAL;
                    break;
                case MANUAL:
                    aimMode = AIM_MODE.VISION;
                    break;
            }
            telemetry.addData("Aim Mode", aimMode);
        }

        // P2 A 按下时：若炮台正在发射则持续给球，否则停止；此模式下不服从操作手的左右 bumper 或 Y 键
        if (gamepad2.a) {
            if (turret.isLaunching()) {
                sweeper.setGiveArtifact();
            } else {
                sweeper.setStop();
            }
        }else {
            // 球满时自动停止，除非操作手按住左右 bumper 强行继续
            boolean driverOverride = gamepad1.left_bumper || gamepad1.right_bumper;
            if (RobotPosition.getInstance().isFull() && !driverOverride) {
                sweeper.setStop();
            } else if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else if (gamepad1.y) {
                sweeper.setStop();
            }
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

                // 手动模式下使用 update(roll, yaw, speed, shouldShoot, preSpeed)
                turret.update(roll, yaw, targetSpeed, isShooting, preSpeed);

            } else {
                // --- VISION / LOCALIZATION 模式：自动瞄准 ---
                boolean allowVision = (aimMode == AIM_MODE.VISION);

                // 预载飞轮速度（P2 右扳机）
                preSpeed = Math.round(gamepad2.right_trigger * HypParams.maxPreSpeed);

                // 发射开关
                isShooting = gamepad2.y;

                turret.update(allowVision, isShooting, targetTagId, preSpeed);
            }

            // ======== Update & Telemetry ========

            sweeper.update();

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());

            // 瞄准模式显示（含视觉降级指示）
            String aimModeDisplay;
            if (aimMode == OffseasonDECODE_Red.AIM_MODE.VISION) {
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
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        chassis.stop();
        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}