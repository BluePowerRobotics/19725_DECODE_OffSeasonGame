package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ChassisTester-VER1", group="Tests")
public class ChassisTester_ver1 extends LinearOpMode {
    public ChassisController_ver1 chassis;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new ChassisController_ver1(hardwareMap,telemetry);
        boolean isRunning = false;
        waitForStart();
        isRunning = true;
        chassis.ChassisStop();
        chassis.ChassisInit();

        while(opModeIsActive()){
            if(gamepad1.xWasPressed()){
                chassis.SwitchHeadMode();
            }
            if(gamepad1.bWasPressed()){
                isRunning = !isRunning;
                if(!isRunning){
                    chassis.ChassisStop();
                    chassis.localization.Localization();
                    chassis.localization.ChassisVelocityTelemetry();
                    chassis.ChassisModeTelemetry();
                    telemetry.addData("isRunning", isRunning);
                    telemetry.update();

                }
            }
            if(gamepad1.aWasPressed()){
                chassis.localization.resetPosition();
            }
            if(isRunning){
                chassis.GamepadCalculator(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
                chassis.ChassisMoving(chassis.driveXTrans,chassis.driveYTrans, chassis.drivethetaTrans);
                chassis.localization.ChassisLocationTelemetry();
                chassis.ChassisPowerTelemetry();
                chassis.localization.Localization();
                chassis.localization.ChassisVelocityTelemetry( );
                chassis.ChassisModeTelemetry();
                telemetry.addData("isRunning", isRunning);
                telemetry.update();


            }







        }
    }
}
