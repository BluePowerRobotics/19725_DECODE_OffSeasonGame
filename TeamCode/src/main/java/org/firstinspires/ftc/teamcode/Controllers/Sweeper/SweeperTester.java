        package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.HypParams;

@TeleOp(name="SweeperTester", group="Tests")
public class SweeperTester extends LinearOpMode {
    Sweeper sweeper;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sweeper = new Sweeper(hardwareMap, telemetry);
        sweeper.setStop();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                sweeper.setEat();
            } else if (gamepad1.bWasPressed()) {
                sweeper.setGiveArtifact();
            } else if (gamepad1.yWasPressed()) {
                sweeper.setOutput();
            } else if (gamepad1.xWasPressed()) {
                sweeper.setStop();
            } else if (gamepad1.dpadUpWasPressed()) {
                sweeper.setTrigger();
            }

            sweeper.update();
            sweeper.setTelemetry();
            telemetry.addData("TriggerVel", HypParams.SweeperTriggerVel);
            telemetry.update();
        }
    }
}
