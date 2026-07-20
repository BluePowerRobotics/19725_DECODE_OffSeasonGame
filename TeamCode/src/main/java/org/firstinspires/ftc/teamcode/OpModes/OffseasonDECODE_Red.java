package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
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
            RobotPosition.getInstance().update();

            // ======== P1 Controls ========

            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad1.xWasPressed()) {
                chassis.exchangeUseNoHeadMode();
            }

            if (gamepad1.bWasPressed()) {   
                RobotPosition.getInstance().ResetPoseTo(HypParams.ResetPoseRed);
                telemetry.addData("ResetPose", "Reset to Red");
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
                } else if (gamepad1.y) {
                    sweeper.setStop();
                }
            }

            // ======== P2 Controls ========

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
                isShooting = gamepad2.y;

                preSpeed = Math.round(gamepad2.right_trigger * HypParams.maxPreSpeed);

                turret.update(roll, yaw, targetSpeed, isShooting, preSpeed);

            } else {
                boolean allowVision = (aimMode == AIM_MODE.VISION);

                preSpeed = Math.round(gamepad2.right_trigger * HypParams.maxPreSpeed);

                isShooting = gamepad2.y;

                turret.update(allowVision, isShooting, targetTagId, preSpeed);
            }

            // ======== Update & Telemetry ========

            sweeper.update();

            telemetry.addData("Team", "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Aim Mode", aimMode);
            telemetry.addData("PreSpeed", "%d", preSpeed);
            telemetry.addData("Roll", "%.2f deg", roll);
            telemetry.addData("Yaw", "%.2f deg", yaw);
            telemetry.addData("Target Speed", "%d", targetSpeed);
            telemetry.addData("Shooting", isShooting ? "ACTIVE" : "IDLE");
            chassis.telemetry();
            sweeper.setTelemetry();
            telemetry.update();
        }

        chassis.stop();
        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}