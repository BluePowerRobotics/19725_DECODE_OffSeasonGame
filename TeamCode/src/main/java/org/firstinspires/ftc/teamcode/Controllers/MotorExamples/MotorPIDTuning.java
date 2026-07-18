package org.firstinspires.ftc.teamcode.Controllers.MotorExamples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * ExampleVeilVasomotor 类是一个示例 TeleOp，展示如何使用 ExampleVoltageOutMotor 控制电机速度
 */
@Config
@TeleOp(name = "MotorPIDTuning", group = "Tests")
public class MotorPIDTuning extends LinearOpMode {
    /** 目标速度 */
    public double targetVelocity = 1000;
    public double k_p = 0.01;
    public double k_i = 0.2;
    public double k_d = 0.05;
    public double maxi=1;
    public int switchmode=0;




    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化遥测，同时输出到Driver Station和Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // 创建ExampleVoltageOutMotor实例，控制名为"SVA"的电机
        VoltageOutMotor flywheel = new VoltageOutMotor(hardwareMap, "motorL", telemetry,false);
        VoltageOutMotor flywheel2 = new VoltageOutMotor(hardwareMap, "motorR", telemetry,true);

        waitForStart();

        // 主循环
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
            if(gamepad1.leftBumperWasPressed()){
                switch(switchmode){
                    case(0):
                        k_p=k_p-0.01;
                        break;
                    case(1):
                        k_i=k_i-0.01;
                        break;
                    case(2):
                        k_d=k_d-0.01;
                        break;

                }
            }
            if(gamepad1.rightBumperWasPressed()){
                switch(switchmode){
                    case(0):
                        k_p=k_p+0.01;
                        break;
                    case(1):
                        k_i=k_i+0.01;
                        break;
                    case(2):
                        k_d=k_d+0.01;
                        break;
                }
            }
            if(gamepad1.bWasPressed()){
                switchmode=switchmode+1;
                if(switchmode==3){switchmode=0;}
            }
            flywheel.kP =k_p;
            flywheel.kI=k_i;
            flywheel.kD=k_d;
            flywheel.maxI=maxi;
            flywheel2.kP =k_p;
            flywheel2.kI=k_i;
            flywheel2.kD=k_d;
            flywheel2.maxI=maxi;


            // 设置目标速度
            flywheel.setTargetVelocity(targetVelocity);
            flywheel2.setTargetVelocity(targetVelocity);

            // 更新电机控制
            flywheel.update();
            flywheel2.update();

            // 按A键停止电机
            if (gamepad1.a) {
                flywheel.setTargetVelocity(0);
                flywheel2.setTargetVelocity(0);
            }

            // 输出目标速度
            telemetry.addData("Velocity", targetVelocity);
            telemetry.addData("leftV",flywheel.getVelocity());
            telemetry.addData("rightV",flywheel2.getVelocity());
            telemetry.addData("kp",k_p);
            telemetry.addData("ki",k_i);
            telemetry.addData("kd",k_d);
            telemetry.addData("switchmode",switchmode);
            telemetry.update();
        }

        // 停止电机
        flywheel.stop();
    }
}
