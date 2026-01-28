package org.firstinspires.ftc.teamcode.Controllers.Sweeper;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name="SweeperTester", group="Tests")
public class SweeperTester extends LinearOpMode {
    Sweeper sweeper;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sweeper = new Sweeper(hardwareMap);
        sweeper.stop();
        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.aWasPressed()){
                sweeper.Eat();
            }else if(gamepad1.bWasPressed()){
                sweeper.GiveArtifact();
            }else if(gamepad1.yWasPressed()){
                sweeper.output();
            }else if(gamepad1.xWasPressed()){
                sweeper.stop();
            }
            telemetry.addData("ForR",sweeper.getFR());
            telemetry.addData("Sweeper Velocity",sweeper.getVel());
            telemetry.addData("Sweeper Power",sweeper.getPower());
            telemetry.update();
        }
    }



}
