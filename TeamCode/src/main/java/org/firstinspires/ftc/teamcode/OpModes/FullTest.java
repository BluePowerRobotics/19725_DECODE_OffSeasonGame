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

@TeleOp(name = "FullTest", group = "Tests")
public class FullTest extends LinearOpMode {
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    // 手动模式参数（MANUAL mode 下使用）
    private double roll = 0.0;
    private double yaw = 50.0;
    private int targetSpeed = 0;
    private boolean isShooting = false;

    // 瞄准模式（P2 X 循环切换）
    private enum AIM_MODE { VISION, LOCALIZATION, MANUAL }
    private AIM_MODE aimMode = AIM_MODE.VISION;

    // 当前目标 AprilTag ID（根据队伍颜色自动选择）
    private int targetTagId;

    // 预载飞轮速度（P2 右扳机控制）
    private int preSpeed = 0;

    // 队伍颜色
    private TeamColor teamColor;

    private static final double ROLL_SPEED = 2.0;
    private static final double YAW_STEP = 5.0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init 阶段：选择队伍颜色 ----
        telemetry.addData("Select Team Color", "");
        telemetry.addData("Press A (Blue)", "ResetPose: (63, 60.7, pi/2)");
        telemetry.addData("Press B (Red)", "ResetPose: (63, -60.7, -pi/2)");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a && !gamepad1.b) {
            idle();
        }

        if (gamepad1.a) {
            teamColor = TeamColor.BLUE;
            targetTagId = 20; // 蓝队球门 AprilTag ID
        } else {
            teamColor = TeamColor.RED;
            targetTagId = 24; // 红队球门 AprilTag ID
        }

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.setGamepads(gamepad1, gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team Color", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("A", "Reset Pose to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red") + " ResetPose");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output + Flywheel Reverse + Trigger Launch");
        telemetry.addData("Y", "Sweeper Stop");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.addData("X", "Cycle Aim Mode: VISION/LOCALIZATION/MANUAL");
        telemetry.addData("Left Stick X", "Turret Roll (MANUAL mode only)");
        telemetry.addData("D-Pad Up/Down", "Yaw +/-5 (MANUAL mode only)");
        telemetry.addData("D-Pad Left/Right", "Speed +/-100 (MANUAL mode only)");
        telemetry.addData("A", "Toggle Shoot");
        telemetry.addData("Right Trigger", "Preload Speed (auto aim mode)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ======== P1 Controls ========

            // 底盘移动（左摇杆 + 右摇杆 X）
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // 切换无头模式
            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            // 重置定位到对应颜色的 ResetPose
            if (gamepad1.aWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
                telemetry.addData("ResetPose", "Reset to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red"));
            }

            // 吸取器控制 + 炮台反转模式
            // 一操右 bumper 按下时：飞轮反转 + sweeper 反转 + 扳机舵机到发射位置
            if (gamepad1.right_bumper) {
                turret.setReverseMode(true);
                sweeper.setOutput();
            }

            // P2 A 按下时：若炮台正在发射则持续给球，否则停止；此模式下不服从操作手的左右 bumper 或 Y 键
            if (gamepad2.a) {
                if (turret.isLaunching()) {
                    sweeper.setGiveArtifact();
                } else {
                    sweeper.setStop();
                }
            } else {
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

            // 右 bumper 释放时恢复炮台正常模式（总是在 sweeper 控制之后执行）
            if (!gamepad1.right_bumper) {
                turret.setReverseMode(false);
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
                isShooting=gamepad2.a;

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
                isShooting=gamepad2.a;

                // 自动瞄准模式下使用 update(AllowVision, shouldShoot, targetTagId, preSpeed)
                turret.update(allowVision, isShooting, targetTagId, preSpeed);
            }

            // ======== 更新 & 遥测 ========

            sweeper.update();

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());

            // 瞄准模式显示（含视觉降级指示）
            String aimModeDisplay;
            if (aimMode == AIM_MODE.VISION) {
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