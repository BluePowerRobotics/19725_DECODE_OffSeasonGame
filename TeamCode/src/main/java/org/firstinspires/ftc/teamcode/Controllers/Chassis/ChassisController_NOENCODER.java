package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChassisController_NOENCODER {
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    private ElapsedTime runtime = new ElapsedTime(); // 计时工具，用于计算采样周期

    HardwareMap hardwareMap;
    Telemetry telemetry;

    IMU imu;

    public boolean NoHeadMode = false;

    // 机械参数
    //TODO: 根据实际底盘参数修改
    private static final double L = 15.0;                 // 底盘前后轮中心纵向距离，单位：cm
    private static final double W = 15.0;                 // 底盘左右轮中心横向距离，单位：cm
    private static final double D = Math.sqrt(L*L+W*W);    // 复合参数，无需修改


    // 控制参数
    //TODO: 根据实际需求调整
    private static final double DRIVE_SPEED = 0.8;        // 最大驱动速度（0~1，防止打滑）
    private static final double TURN_SPEED = 0.6;         // 最大旋转速度（0~1）

    // 电机端口配置
    //TODO: 根据实际接线修改
    private static final String FL_MOTOR = "frontLeft";  // 前左电机端口名
    private static final String FR_MOTOR = "frontRight"; // 前右电机端口名
    private static final String BL_MOTOR = "backLeft";   // 后左电机端口名
    private static final String BR_MOTOR = "backRight";  // 后右电机端口名
    //手柄解算参数
    public double driveXTrans = 0.0;
    public double driveYTrans = 0.0;
    public double drivethetaTrans= 0.0;

    public double theta=0.0;

    public ChassisController_NOENCODER(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class, FL_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, FR_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, BL_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, BR_MOTOR);
        imu = hardwareMap.get(IMU.class, "imu");

        this.telemetry = telemetry;

    }
    public void ChassisInit(){
        // 右斜麦克纳姆轮标准配置：前右/后右电机反转，前左/后左正转（可根据实际运动调整）
        //TODO: 根据实际运动调整电机方向
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // 编码器模式配置：重置并设置为增量模式
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //自锁模式配置
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));


    }

    /**
     *
     * @param x
     * @param y
     * @param theta
     *
     * 底盘运动控制，包含telemetry
     * frontleftpower
     * frontrightpowe
     * backleftpower
     * backrightpower
     */
    public void ChassisMoving(double x, double y,double theta){
        double flPower = x + y + theta*D;
        double frPower = -x + y - theta*D;
        double blPower = -x + y + theta*D;
        double brPower = x + y - theta*D;

        double maxPower = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));
        flPower /= maxPower;
        frPower /= maxPower;
        blPower /= maxPower;
        brPower /= maxPower;

        ChassisPowerTelemetry(flPower, frPower, blPower, brPower);
        setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public void ChassisPowerTelemetry(double fl, double fr ,double bl, double br) {
        telemetry.addData("Front Left Power: ", fl);
        telemetry.addData("Front Right Power: ", fr);
        telemetry.addData("Back Left Power: ", bl);
        telemetry.addData("Back Right Power: ", br);
    }
    private void setMotorPowers(double fl, double fr, double rl, double rr) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(rl);
        backRight.setPower(rr);
    }


    public void ChassisStop(){
        ChassisPowerTelemetry(0,0,0,0);
        setMotorPowers(0,0,0,0);
    }



    public void GamepadCalculator(double gamepad_x, double gamepad_y,double gamepad_theta){
        double x = gamepad_x*DRIVE_SPEED;
        double y = gamepad_y*DRIVE_SPEED;
        double theta = gamepad_theta*TURN_SPEED;
        driveXTrans = x;
        driveYTrans = y;
        drivethetaTrans  = theta;

        if (NoHeadMode) {
            driveXTrans = x * Math.cos(this.theta) + y * Math.sin(this.theta);
            driveYTrans = -x * Math.sin(this.theta) + y * Math.cos(this.theta);
        }

    }

    public void SwitchHeadMode(){
        NoHeadMode = !NoHeadMode;
    }
    public void resetPosition(){

        theta = 0.0;
    }

    public void setIMU(){
        theta=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void ChassisModeTelemetry(){
        telemetry.addData("NoHeadMode: ", NoHeadMode);
        telemetry.addData("movingspeed",DRIVE_SPEED);
        telemetry.addData("turnspeed",TURN_SPEED);

    }

}
