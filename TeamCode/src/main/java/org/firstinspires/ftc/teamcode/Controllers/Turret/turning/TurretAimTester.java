package org.firstinspires.ftc.teamcode.Controllers.Turret.turning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.ChassisController_ver1;

@TeleOp(name = "TurretAimTester", group = "Tests")
public class TurretAimTester extends LinearOpMode {
    TurretAimController turretAimController;
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

        turretAimController = new TurretAimController(hardwareMap, "shootingMotor");
        waitForStart();
        isRunning = true;
        chassis.ChassisStop();
        chassis.ChassisInit();


        while (opModeIsActive()) {
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
                chassis.localization.Localization();
                chassis.localization.ChassisVelocityTelemetry( );
                chassis.ChassisModeTelemetry();
                telemetry.addData("isRunning", isRunning);
                telemetry.update();


            }
            // 示例：按下游戏手柄按钮设置目标位置
            if(gamepad1.yWasPressed()){

                turretAimController.setTargetPosition(100.0, 100.0);
            }

            // 更新炮台瞄准控制
            turretAimController.update(chassis.localization.x,chassis.localization.y,chassis.localization.theta);
            // 添加其他测试逻辑和数据输出
            telemetry.addData("Turret Angle", turretAimController.getTurretAngle());
        }
    }


}
