package org.firstinspires.ftc.teamcode.Controllers.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Shootertester", group = "Tests")
public class Shootertester extends LinearOpMode {

    public Shooter shooter;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter = new Shooter(hardwareMap, telemetry,"shooter",true);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                shooter.shoot(850);
            } else if (gamepad1.bWasPressed()) {
                shooter.shoot(760);
            } else if (gamepad1.yWasPressed()) {
                shooter.shoot(650);
            } else if (gamepad1.xWasPressed()) {
                shooter.shoot(0);
            }
            shooter.setTelemetry();
            telemetry.update();
        }
    }
}