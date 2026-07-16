package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooterstester", group = "Tests")
public class ShooterSmallTester extends LinearOpMode {

    DcMotor shooterL;
    DcMotor shooterR;

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.shooterL = hardwareMap.get(DcMotorEx.class, "motorL");
        this.shooterR = hardwareMap.get(DcMotorEx.class, "motorR");


        // 配置电机
        this.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.shooterL.setDirection(DcMotorEx.Direction.FORWARD);

        this.shooterR.setDirection(DcMotorEx.Direction.REVERSE);

        // 初始化飞轮系统，根据实际电机名称和反转设置调整
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpadUpWasPressed()){
                shooterL.setPower(0.7);
                shooterR.setPower(0.7);
            }
            if(gamepad1.dpadDownWasPressed()){
                shooterL.setPower(0);
                shooterR.setPower(0);
            }


        }
    }
}