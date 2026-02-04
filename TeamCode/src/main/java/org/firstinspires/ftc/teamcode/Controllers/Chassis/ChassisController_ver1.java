package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ChassisController_ver1 {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public boolean NoHeadMode = false;

    // 控制参数
    public static double DRIVE_SPEED = 0.8;        // 最大驱动速度（0~1，防止打滑）
    public static double TURN_SPEED = 0.6;         // 最大旋转速度（0~1）

    // 电机端口配置
    private static final String FL_MOTOR = "fL";  // 前左电机端口名
    private static final String FR_MOTOR = "fR"; // 前右电机端口名
    private static final String BL_MOTOR = "bL";   // 后左电机端口名
    private static final String BR_MOTOR = "bR";  // 后右电机端口名

    //电机速度参数
    public double flPower=0;
    public double frPower=0;
    public double blPower=0;
    public double brPower=0;

    //手柄解算参数
    public double driveXTrans = 0.0;
    public double driveYTrans = 0.0;
    public double drivethetaTrans= 0.0;

    // 本控制器持有定位子模块
    public ChassisLocalization_Encoder localization;

    public ChassisController_ver1(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotorEx.class, FL_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, FR_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, BL_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, BR_MOTOR);

        this.telemetry = telemetry;

        // 初始化定位模块（传入电机与 telemetry）
        this.localization = new ChassisLocalization_Encoder(frontLeft, frontRight, backLeft, backRight, telemetry);
    }
    public void ChassisInit(){
        // 右斜麦克纳姆轮标准配置：前右/后右电机反转，前左/后左正转（可根据实际运动调整）
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

        // 初始化定位模块（重置计时与初始编码器/位姿）
        localization.init();
    }

    /**
     *
     * @param x
     * @param y
     * @param theta
     *
     * 底盘运动控制，包含telemetry
     */
    public void ChassisMoving(double x, double y,double theta){
        flPower = x + y + theta;
        frPower = -x + y - theta;
        blPower = -x + y + theta;
        brPower = x + y - theta;

        double maxPower = Math.max(1.0, Math.max(Math.abs(flPower),Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));
        flPower /= maxPower;
        frPower /= maxPower;
        blPower /= maxPower;
        brPower /= maxPower;

        setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public void ChassisPowerTelemetry() {
        telemetry.addData("Front Left Power: ", flPower);
        telemetry.addData("Front Right Power: ", frPower);
        telemetry.addData("Back Left Power: ", blPower);
        telemetry.addData("Back Right Power: ", brPower);
    }
    private void setMotorPowers(double fl, double fr, double rl, double rr) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(rl);
        backRight.setPower(rr);
    }

    public void ChassisStop(){
        flPower=0;
        frPower=0;
        blPower=0;
        brPower=0;
        ChassisPowerTelemetry();
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
            double robotTheta = localization.getTheta();
            driveXTrans = x * Math.cos(robotTheta) + y * Math.sin(robotTheta);
            driveYTrans = -x * Math.sin(robotTheta) + y * Math.cos(robotTheta);
        }

    }
    public void SwitchHeadMode(){
        NoHeadMode = !NoHeadMode;
    }

    public void ChassisModeTelemetry(){
        telemetry.addData("NoHeadMode: ", NoHeadMode);
        telemetry.addData("movingspeed",DRIVE_SPEED);
        telemetry.addData("turnspeed",TURN_SPEED);
    }

}