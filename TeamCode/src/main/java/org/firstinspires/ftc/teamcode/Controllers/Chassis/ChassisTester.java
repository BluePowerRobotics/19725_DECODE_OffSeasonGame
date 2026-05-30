package org.firstinspires.ftc.teamcode.Controllers.Chassis;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.TeamColor;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@TeleOp(name="ChassisTester", group="Tests")
public class ChassisTester extends LinearOpMode {

    long lastNanoTime=0;
    boolean useNoHeadMode = false;


    @Override
    public void runOpMode()throws InterruptedException{
        ActionRunner actionRunner = new ActionRunner();
        Chassis chassis=new Chassis(hardwareMap, TeamColor.RED, actionRunner, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            RobotPosition.getInstance().update();
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, useNoHeadMode);
            if(gamepad1.xWasReleased()) useNoHeadMode = !useNoHeadMode;
            lastNanoTime=System.nanoTime();
            telemetry.addData("NoHeadMode", useNoHeadMode);
            chassis.telemetry();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
