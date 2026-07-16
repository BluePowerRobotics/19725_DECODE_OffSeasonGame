package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Shootertester", group = "Tests")
public class Shootertester extends LinearOpMode {

    public Shooter shooter;
    public int targetVelocity=1000;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 初始化飞轮系统，根据实际电机名称和反转设置调整
        shooter = new Shooter(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpadUpWasPressed()){
                targetVelocity=targetVelocity+100;
            }
            if(gamepad1.dpadDownWasPressed()){
                targetVelocity=targetVelocity-100;
            }
            if(gamepad1.dpadLeftWasPressed()){
                targetVelocity=targetVelocity-500;
            }
            if(gamepad1.dpadRightWasPressed()){
                targetVelocity=targetVelocity+500;
            }
            shooter.setTargetVelocity(targetVelocity); // 停止

            shooter.update();
            telemetry.addData("Velocity", targetVelocity);
            telemetry.addData("leftV",shooter.getSpeedL());
            telemetry.addData("rightV",shooter.getSpeedR());


            telemetry.update();
        }
    }
}