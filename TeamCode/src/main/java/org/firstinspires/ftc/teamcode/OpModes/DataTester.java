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

import java.util.ArrayList;
import java.util.List;

/**
 * 自动标定程序：用于测试 data_withouttime.csv 的射速-距离关系
 *
 * 操作说明：
 *   P1 左摇杆 + 右摇杆X：底盘移动
 *   P1 左 Bumper：吸球
 *   P1 方向键上/下：手动调整仰角 (yaw)
 *   P1 方向键左/右：手动调整射速 (speed)
 *   P1 A 按住：发射（飞轮加速到目标速度后自动释放扳机）
 *   P1 右 Bumper：强制释放扳机
 *   P1 B 按下：记录一组数据 (yaw, speed, distance)
 *   P1 X：切换无头模式
 *   P1 Y：停止吸球
 */
@TeleOp(name = "DataTester", group = "Tests")
public class DataTester extends LinearOpMode {

    private Chassis chassis;
    private Turret turret;
    private Sweeper sweeper;

    // 手动参数
    private double manualYaw = 50;
    private int manualSpeed = 1500;

    private static final double YAW_STEP = 5.0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 3000;

    // 发射状态机（达速后持续 cycle: launch → reset → launch → ...）
    private boolean shooting = false;
    private boolean triggerAtLaunch = false; // true=扳机在发射位置, false=扳机在复位位置
    private long cycleTime = 0;
    private static final long TRIGGER_LAUNCH_HOLD_MS = 300; // 扳机发射位置保持时间
    private static final long TRIGGER_RESET_HOLD_MS = 200;  // 扳机复位位置保持时间

    // 瞄准信息
    private double currentDistance = 0;
    private double aimRoll = 0;
    private boolean targetFound = false;

    // 记录数据
    private final List<String> recordedData = new ArrayList<>();

    // 队伍颜色（可在此修改为 BLUE 测试蓝方）
    private final TeamColor teamColor = TeamColor.BLUE;
    private int targetTagId;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, true);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.setTargetTagId(20);
        turret.setGamepads(gamepad1, gamepad2);

        telemetry.addLine("=== DataTester ===");
        telemetry.addLine("D-pad U/D: yaw  D-pad L/R: speed");
        telemetry.addLine("A: shoot  B: record  LB: intake");
        telemetry.addLine("RB: force launch  X: switch heading mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ======== 底盘控制 ========
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            // ======== 吸球控制 ========
            if (gamepad1.left_bumper) {
                sweeper.setEat();
            } else if (gamepad1.y) {
                sweeper.setStop();
            }

            // ======== 炮台自动瞄准 + 测距 ========
            Object[] aimResult = turret.aim(true);
            targetFound = (boolean) aimResult[0];
            aimRoll = (double) aimResult[1];
            double aimYaw = (double) aimResult[2];

            if (targetFound) {
                double cotYaw = 1.0 / Math.tan(Math.toRadians(aimYaw));
                currentDistance = HypParams.TagH * cotYaw + HypParams.WebCamCenterDistance;
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
                    currentDistance = Math.hypot(relativeDx, relativeDy);
                }
            }

            // 覆盖仰角为手动值（aim() 已设置 roll 对准球门，此处覆盖 yaw）
            turret.rotate_to(aimRoll, manualYaw);

            // ======== 方向键：手动调整仰角和速度 ========
            if (gamepad1.dpadUpWasPressed()) {
                manualYaw += YAW_STEP;
            }
            if (gamepad1.dpadDownWasPressed()) {
                manualYaw -= YAW_STEP;
            }
            if (gamepad1.dpadRightWasPressed()) {
                manualSpeed += SPEED_STEP;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                manualSpeed -= SPEED_STEP;
            }
            manualSpeed = Math.max(SPEED_MIN, Math.min(SPEED_MAX, manualSpeed));

            // ======== 发射逻辑 ========
            boolean shootRequested = gamepad1.a;

            // 强制发射
            if (gamepad1.right_bumper) {
                turret.forceLaunch();
                triggerAtLaunch = true;
                cycleTime = System.currentTimeMillis();
            }

            if (shootRequested) {
                if (!shooting) {
                    // 开始发射：设置飞轮目标速度
                    shooting = true;
                    triggerAtLaunch = false;
                    cycleTime = System.currentTimeMillis();
                    turret.shooter.setTargetVelocity(manualSpeed);
                }

                turret.shooter.update();

                // 达速后持续 cycle: launch → reset → launch → ...
                if (turret.shooter.reachedVelocity()) {
                    turret.launch();

                    sweeper.setGiveArtifact();
                }
            } else {
                // 松 A 停止发射
                if (shooting) {
                    shooting = false;
                    triggerAtLaunch = false;
                    turret.reset();
                    turret.shooter.setTargetVelocity(0);
                    turret.shooter.update();
                    sweeper.setStop();
                }
            }

            // 飞轮始终更新（即使不在发射状态也要更新以保持速度为零）
            if (!shooting) {
                turret.shooter.update();
            }

            // 炮台 PID 更新
            turret.turretDegreeController.update();

            // ======== B 键：记录数据 ========
            if (gamepad1.bWasPressed()) {
                int currentSpeed = (int) turret.shooter.getCurrentVelocity();
                String record = String.format("%.0f, %d, %.1f", manualYaw, currentSpeed, currentDistance);
                recordedData.add(record);
                telemetry.addData("Recorded[" + (recordedData.size() - 1) + "]", record);
            }

            // ======== 吸球更新 ========
            sweeper.update();

            // ======== Telemetry ========
            double[] angles = turret.get_angle();

            telemetry.addData("Team", teamColor);
            telemetry.addData("Target Found", targetFound ? "YES" : "NO (LOCALIZATION)");
            telemetry.addData("Distance", "%.1f in", currentDistance);

            telemetry.addLine("--- Manual Params ---");
            telemetry.addData("Manual Yaw", "%.1f deg", manualYaw);
            telemetry.addData("Manual Speed", "%d RPM", manualSpeed);

            telemetry.addLine("--- Flywheel ---");
            telemetry.addData("Target Speed", "%.0f RPM", turret.shooter.getTargetVelocity());
            telemetry.addData("Current Speed", "%.0f RPM", turret.shooter.getCurrentVelocity());
            telemetry.addData("Shooting", shooting ? (triggerAtLaunch ? "LAUNCH" : "RESET") : "IDLE");

            telemetry.addLine("--- Turret ---");
            telemetry.addData("Turret Roll", "%.2f deg", angles[0]);
            telemetry.addData("Turret Yaw", "%.2f deg", angles[1]);

            telemetry.addLine("--- Intake ---");
            telemetry.addData("Intake Vel", "%.0f RPM", sweeper.getVel());

            telemetry.addLine("--- Pose ---");
            telemetry.addData("X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            // 已记录数据
            telemetry.addLine("--- Recorded Data (" + recordedData.size() + " entries) ---");
            int displayStart = Math.max(0, recordedData.size() - 10);
            for (int i = displayStart; i < recordedData.size(); i++) {
                telemetry.addData("[" + i + "]", recordedData.get(i));
            }

            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // 停止
        chassis.stop();
        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}