package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;

@Config
@TeleOp(name = "ChassisTester", group = "Tests")
public class ChassisTester extends LinearOpMode {
    long lastNanoTime=0;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotPosition.getInstance().update();
        ChassisController chassis = new ChassisController(hardwareMap,new Pose2d(0,0,0));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        lastNanoTime=System.nanoTime();
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y-gamepad1.right_stick_y; // 前后
            double strafe = gamepad1.left_stick_x; // 左右
            double rotate =-gamepad1.right_stick_x; // 旋转——正为逆时针旋转
            chassis.gamepadInput(strafe, drive, rotate);
            if(gamepad1.xWasReleased()) chassis.exchangeNoHeadMode();

            telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
            telemetry.addData("y-power",drive);
            telemetry.addData("x-power",strafe);
            telemetry.addData("r-power",rotate);
            telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
            telemetry.addData("NoHeadMode",chassis.getUseNoHeadMode()?"NoHead":"Manual");
            telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");//chassis.isTargetPointReached()
            telemetry.addData("Position",RobotPosition.getInstance().getPose().toString());
            telemetry.update();
            lastNanoTime = System.nanoTime();


            Pose2d pose = RobotPosition.getInstance().getPose();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}