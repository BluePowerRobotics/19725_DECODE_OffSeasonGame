package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.utility.HypParams;

@TeleOp(name = "Shooter Intake Tester", group = "Tests")
public class ShooterIntaketester extends LinearOpMode {

    public Shooter shooter;
    public int targetVelocity = 1700;

    private Sweeper sweeper;
    private int sweeperTargetVelocity = 1000;

    private Servo triggerServo;
    private boolean triggerAtLaunch = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize shooter
        shooter = new Shooter(hardwareMap, telemetry);

        // Initialize sweeper
        sweeper = new Sweeper(hardwareMap, telemetry);
        sweeper.setStop();

        // Initialize trigger servo
        triggerServo = hardwareMap.get(Servo.class, "trigger");
        triggerServo.setPosition(HypParams.triggerLaunchPosition);
        triggerAtLaunch = false;

        telemetry.addLine("=== Shooter Intake Tester ===");
        telemetry.addLine("D-pad Up/Down: shooter speed +/-100");
        telemetry.addLine("D-pad Left/Right: intake speed +/-100");
        telemetry.addLine("Left Bumper: trigger launch");
        telemetry.addLine("Right Bumper: trigger reset");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- Shooter velocity control (D-pad) ----
            if (gamepad1.dpadUpWasPressed()) {
                targetVelocity += 100;
            }
            if (gamepad1.dpadDownWasPressed()) {
                targetVelocity -= 100;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                sweeperTargetVelocity -= 100;
            }
            if (gamepad1.dpadRightWasPressed()) {
                sweeperTargetVelocity += 100;
            }
            shooter.setTargetVelocity(targetVelocity);

            // ---- Sweeper velocity control (D-pad Left/Right) ----
            sweeper.setTargetVelocity(sweeperTargetVelocity);

            // ---- Trigger servo control (bumpers) ----
            if (gamepad1.left_bumper) {
                triggerServo.setPosition(HypParams.triggerLaunchPosition);
                triggerAtLaunch = true;
            } else if (gamepad1.right_bumper) {
                triggerServo.setPosition(HypParams.triggerResetPosition);
                triggerAtLaunch = false;
            }

            // Update subsystems
            shooter.update();
            sweeper.update();

            // ---- Telemetry ----
            telemetry.addLine("====== Shooter ======");
            telemetry.addData("Target Velocity", "%d RPM", targetVelocity);
            telemetry.addData("Left Vel", "%.0f RPM", shooter.getSpeedL());
            telemetry.addData("Right Vel", "%.0f RPM", shooter.getSpeedR());

            telemetry.addLine();
            telemetry.addLine("====== Sweeper (Intake) ======");
            telemetry.addData("Target Velocity", "%d RPM", sweeperTargetVelocity);
            telemetry.addData("Actual Velocity", "%.0f RPM", sweeper.getVel());

            telemetry.addLine();
            telemetry.addLine("====== Trigger ======");
            telemetry.addData("Position", triggerAtLaunch ? "LAUNCH (%.3f)" : "RESET (%.3f)",
                    triggerAtLaunch ? HypParams.triggerLaunchPosition : HypParams.triggerResetPosition);

            telemetry.update();
        }

        // Stop
        shooter.setTargetVelocity(0);
        sweeper.setStop();
        sweeper.update();
        triggerServo.setPosition(HypParams.triggerResetPosition);
    }
}