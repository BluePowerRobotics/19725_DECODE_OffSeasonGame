package org.firstinspires.ftc.teamcode.Controllers.Chassis;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;

@TeleOp(name="ChassisTester_limelight", group="Tests")
public class ChassisTester_limelighttuning extends LinearOpMode {
    public ChassisController chassis;
    public Limelight3A limelight;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        chassis = new ChassisController(hardwareMap,telemetry);
        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        boolean isRunning = false;
        waitForStart();
        isRunning = true;

        chassis.ChassisStop();
        chassis.ChassisInit();

        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            Pose3D botpose= result.getBotpose();
            telemetry.addData("X",result.getTx());
            telemetry.addData("Y",result.getTy());
            telemetry.addData("a",result.getTa());
            telemetry.addData("Botpose",botpose.toString());




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


                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");

                Drawing.drawRobot(packet.fieldOverlay(), this.toPose2D(botpose));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);



            }







        }
    }
    public Pose2d toPose2D(Pose3D pose3D) {
        Position position= pose3D.getPosition();
        YawPitchRollAngles orientation= pose3D.getOrientation();
        // 空值安全处理：如果position或orientation为空，返回原点Pose2D
        if (position == null || orientation == null) {
            return new      Pose2d(0, 0, 0);
        }
        

        // 提取平面平移量（X、Y），忽略Z轴
        double x = 100*position.x;
        double y = 100*position.y;

        // 提取绕Z轴的旋转角（Yaw），忽略Roll和Pitch
        double yaw = orientation.getYaw(AngleUnit.DEGREES); // 默认使用角度单位

        // 构建并返回Pose2D
        return new Pose2d(x, y, Math.toRadians(yaw));
    }
}
