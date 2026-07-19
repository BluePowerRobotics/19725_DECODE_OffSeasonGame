package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TurretTickTestS", group = "Tests")
public class TurretTickTest extends LinearOpMode {
    DcMotorEx roll;
    Servo yaw;
    double MOTOR_TICK_PER_DEGREE;
    double SERVO_POSITION_PER_DEGREE = 0.0385;

    double targetPosition_r =0;
    double targetPosition_y =0;

    double currentDegree_r =0;
    double currentDegree_y =0;
    boolean mode=true;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        roll = hardwareMap.get(DcMotorEx.class,"roll");
        roll.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        roll.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        yaw = hardwareMap.get(Servo.class,"yaw");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.aWasReleased()){
                mode=!mode;
            }
            if(mode){
                if(gamepad1.dpadUpWasPressed()){
                    targetPosition_r = targetPosition_r +80;
                }
                if(gamepad1.dpadDownWasPressed()){
                    targetPosition_r = targetPosition_r -80;
                }
                if(gamepad1.dpadLeftWasPressed()){
                    currentDegree_r = currentDegree_r -1;
                }
                if(gamepad1.dpadRightWasPressed()){
                    currentDegree_r = currentDegree_r +1;
                }
                roll.setTargetPosition((int) targetPosition_r);
                roll.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                roll.setPower(0.25);
                if(gamepad1.bWasPressed()){MOTOR_TICK_PER_DEGREE=roll.getCurrentPosition()/ currentDegree_r;}


            }
            else{
                if(gamepad1.dpadUpWasPressed()){
                    targetPosition_y=targetPosition_y+0.05;
                }
                if(gamepad1.dpadDownWasPressed()){
                    targetPosition_y=targetPosition_y-0.05;
                }
                if(gamepad1.dpadLeftWasPressed()){
                    currentDegree_y = currentDegree_y -1;
                }
                if(gamepad1.dpadRightWasPressed()){
                    currentDegree_y = currentDegree_y +1;
                }
                yaw.setPosition(targetPosition_y);
                if(gamepad1.bWasReleased()){SERVO_POSITION_PER_DEGREE=targetPosition_y/ currentDegree_y;}


            }

            telemetry.addData("Target Position_r", targetPosition_r);
            telemetry.addData("Current Position_r",roll.getCurrentPosition());
            telemetry.addData("Current Degree_r", currentDegree_r);
            telemetry.addData("Motor Tick Per Degree",MOTOR_TICK_PER_DEGREE);
            telemetry.addData("Target Position_y",targetPosition_y);
            telemetry.addData("Current Position_y",yaw.getPosition());
            telemetry.addData("Current Degree_y", currentDegree_y);
            telemetry.addData("Servo Position Per Degree",SERVO_POSITION_PER_DEGREE);
            telemetry.update();

        }

    }
}
