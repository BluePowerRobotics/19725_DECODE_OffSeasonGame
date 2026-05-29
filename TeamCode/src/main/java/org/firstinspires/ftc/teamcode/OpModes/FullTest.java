package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.turner.TurretDegreeController;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;

@TeleOp(name = "FullTest", group = "Tests")
public class FullTest extends LinearOpMode {
    private Chassis chassis;
    private Sweeper sweeper;
    private Shooter shooter;
    private Turret turret;
    private boolean useNoHeadMode = false;
    private boolean isShooting = false;
    private boolean prepareDone = false;
    private int targetSpeed = 600;
    private double targetYaw = TurretDegreeController.YAW_OFFSET;
    private double manualRollTarget = 0.0;
    private static final double ROLL_SPEED = 3.0;
    private static final double YAW_STEP = 2.0;
    private static final int SPEED_STEP = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        ActionRunner actionRunner = new ActionRunner();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        sweeper = new Sweeper(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        chassis = new Chassis(hardwareMap, OffseasonDECODE.TEAM_COLOR.RED, actionRunner, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ==================== 一操：底盘控制 ====================
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, useNoHeadMode);
            if (gamepad1.xWasReleased()) useNoHeadMode = !useNoHeadMode;

            // ==================== Sweeper 控制 ====================
            // sweeper优先听从二操：isShooting 时二操控制，否则一操控制
            if (isShooting) {
                shooter.setTargetVelocity(targetSpeed);
                if (shooter.reachedVelocity()) {
                    sweeper.setTrigger();
                } else if (!sweeper.isPreparing() && !prepareDone) {
                    sweeper.prepare(HypParams.PrepareAngle);
                    prepareDone = true;
                }
            } else {
                if (gamepad1.left_bumper) {
                    sweeper.setEat();
                } else if (gamepad1.right_bumper) {
                    sweeper.setOutput();
                } else if (gamepad1.yWasReleased()) {
                    sweeper.setStop();
                }
            }

            // ==================== 二操：炮台控制 ====================
            // 左摇杆 → roll（水平旋转，增量式位置控制）
            double stickX = gamepad2.left_stick_x;
            if (Math.abs(stickX) > 0.1) {
                manualRollTarget += stickX * ROLL_SPEED;
            }
            turret.turretDegreeController.setTargetRoll(manualRollTarget);

            // D-pad 上下 → yaw（仰角）
            if (gamepad2.dpad_up) {
                targetYaw = Math.min(TurretDegreeController.YAW_ANGLE_MAX, targetYaw + YAW_STEP);
            }
            if (gamepad2.dpad_down) {
                targetYaw = Math.max(TurretDegreeController.YAW_ANGLE_MIN, targetYaw - YAW_STEP);
            }
            turret.turretDegreeController.setTargetYaw(targetYaw);

            // D-pad 左右 → 预期射速（仅设置预期功率，飞轮仍停止）
            if (gamepad2.dpad_right) {
                targetSpeed += SPEED_STEP;
            } else if (gamepad2.dpadLeftWasPressed()) {
                targetSpeed -= SPEED_STEP;
            }

            // A 键 → 发射切换（同时启动飞轮和操作 sweeper）
            if (gamepad2.aWasPressed()) {
                isShooting = !isShooting;
                if (isShooting) {
                    prepareDone = false;
                } else {
                    shooter.setTargetVelocity(0);
                    sweeper.setStop();
                }
            }

            // ==================== 更新所有部件 ====================
            turret.turretDegreeController.update();
            shooter.update();
            sweeper.update();

            // ==================== 遥测 ====================
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("NoHeadMode", useNoHeadMode);
            telemetry.addData("TargetSpeed", targetSpeed);
            telemetry.addData("TargetYaw", targetYaw);
            telemetry.addData("ManualRoll", manualRollTarget);
            telemetry.addData("IsShooting", isShooting);
            telemetry.addData("ShooterReady", shooter.reachedVelocity());
            telemetry.addData("SweeperPreparing", sweeper.isPreparing());
            telemetry.addData("ShootPhase", turret.getShootPhase());
            sweeper.setTelemetry();
            shooter.setTelemetry();
            chassis.telemetry();
            telemetry.update();
        }
    }
}