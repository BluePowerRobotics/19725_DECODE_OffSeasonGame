package org.firstinspires.ftc.teamcode.Controllers.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Shootertester", group = "Tests")
public class Shootertester extends LinearOpMode {
    public ShooterAction shooterAction;
    public Telemetry telemetryrc;
    public void runOpMode() throws InterruptedException {
        shooterAction = new ShooterAction(hardwareMap, telemetryrc);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                shooterAction.setShootSpeed(850);
            } else if (gamepad1.bWasPressed()) {
                shooterAction.setShootSpeed(760);
            } else if (gamepad1.yWasPressed()) {
                shooterAction.setShootSpeed(650);
            } else if (gamepad1.xWasPressed()) {
                shooterAction.setShootSpeed(0);
            }
            shooterAction.setTelemetry();
            telemetry.update();
        }
    }



 }
