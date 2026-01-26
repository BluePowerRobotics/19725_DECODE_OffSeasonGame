package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ChassisTester_NOENCODER", group="Tests")
public class ChassisTester_NOENCODER extends LinearOpMode {
    ChassisController_NOENCODER chassis;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap = this.hardwareMap;
        telemetry = this.telemetry;
        chassis = new ChassisController_NOENCODER(hardwareMap,telemetry);
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
                }
            }
            if(gamepad1.aWasPressed()){
                chassis.resetPosition();
            }
            if(isRunning){
                chassis.GamepadCalculator(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
                chassis.ChassisMoving(chassis.driveXTrans,chassis.driveYTrans, chassis.drivethetaTrans);
                chassis.ChassisModeTelemetry();
                telemetry.addData("isRunning", isRunning);
                telemetry.update();


            }

        }
    }
}
