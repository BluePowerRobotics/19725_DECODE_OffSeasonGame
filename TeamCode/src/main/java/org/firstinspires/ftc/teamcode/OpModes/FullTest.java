package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@TeleOp(name = "FullTest", group = "Tests")
public class FullTest extends LinearOpMode {
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    private double roll = 0.0;
    private double yaw = 50.0;
    private int targetSpeed = 0;
    private boolean isShooting = false;

    private static final double ROLL_SPEED = 2.0;
    private static final double YAW_STEP = 5.0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, OffseasonDECODE.TEAM_COLOR.RED, actionRunner, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("Y", "Sweeper Stop");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.addData("Left Stick X", "Turret Roll (incremental)");
        telemetry.addData("D-Pad Up/Down", "Yaw Adjustment");
        telemetry.addData("D-Pad Left/Right", "Speed Adjustment");
        telemetry.addData("A", "Toggle Shoot");
        telemetry.addData("Left Bumper", "Sweeper Eat (Priority)");
        telemetry.addData("Right Bumper", "Sweeper Output (Priority)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.xWasPressed()) {
                chassis.exchangeUseNoHeadMode();
            }

            if (isShooting) {
                sweeper.setGiveArtifact();
            } else {
                boolean g2SweeperActive = false;
                if (gamepad2.left_bumper) {
                    sweeper.setEat();
                    g2SweeperActive = true;
                } else if (gamepad2.right_bumper) {
                    sweeper.setOutput();
                    g2SweeperActive = true;
                }

                if (!g2SweeperActive) {
                    if (gamepad1.left_bumper) {
                        sweeper.setEat();
                    } else if (gamepad1.right_bumper) {
                        sweeper.setOutput();
                    } else if (gamepad1.y) {
                        sweeper.setStop();
                    }
                }
            }

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

            if (gamepad2.aWasPressed()) {
                isShooting = !isShooting;
                telemetry.addData("Shooting Toggle", isShooting ? "ON" : "OFF");
            }

            turret.update(roll, yaw, targetSpeed, isShooting);

            sweeper.update();

            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Roll", "%.2f deg", roll);
            telemetry.addData("Yaw", "%.2f deg", yaw);
            telemetry.addData("Target Speed", "%d RPM", targetSpeed);
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