package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config

public class ChassisController {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private ElapsedTime runtime = new ElapsedTime(); // 计时工具，用于计算采样周期

    HardwareMap hardwareMap;
    Telemetry telemetry;
//    IMU imu;

    public boolean NoHeadMode = false;

    // 机械参数
    //TODO: 根据实际底盘参数修改
    public static double L = 15.0;                 // 底盘前后轮中心纵向距离，单位：cm
    public static double W = 15.0;                 // 底盘左右轮中心横向距离，单位：cm
    public static double D = Math.sqrt(L*L+W*W);    // 复合参数，无需修改
    public static double GEAR_RATIO = 19.2;         // 减速比（无减速箱则为1，有则填实际值，如50:1则为50）
    public static int ENCODER_PPR = 1024;          // 电机编码器线数（PPR），正交输出可×4（如1024×4=4096）
    public static double WHEEL_RADIUS_CM = 5.0;        // 车轮半径，单位：cm

    // 控制参数
    //TODO: 根据实际需求调整
    public static double DRIVE_SPEED = 0.8;        // 最大驱动速度（0~1，防止打滑）
    public static double TURN_SPEED = 0.6;         // 最大旋转速度（0~1）

    // 电机端口配置
    //TODO: 根据实际接线修改

    private static final String FL_MOTOR = "fL";  // 前左电机端口名
    private static final String FR_MOTOR = "fR"; // 前右电机端口名
    private static final String BL_MOTOR = "bL";   // 后左电机端口名
    private static final String BR_MOTOR = "bR";  // 后右电机端口名

    public int lastFL_enc = 0, lastFR_enc = 0, lastBL_enc = 0, lastBR_enc = 0;

    //电机速度参数
    public double flPower=0;
    public double frPower=0;
    public double blPower=0;
    public double brPower=0;





    // 定位参数
    public double lastTime = 0.0;                       // 上一次采样时间，单位：s
    // 底盘全局位姿（x:X轴坐标 cm，y:Y轴坐标 cm，theta:航向角 rad，初始为原点）
    public double x = 0.0, y = 0.0, theta = 0.0;

    //手柄解算参数
    public double driveXTrans = 0.0;
    public double driveYTrans = 0.0;
    public double drivethetaTrans= 0.0;


    public ChassisController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotorEx.class, FL_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, FR_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, BL_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, BR_MOTOR);
//        imu = hardwareMap.get(IMU.class, "imu");


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



        // 初始化计时和编码器初始值
        runtime.reset();
        lastTime = runtime.seconds();
        lastFL_enc = frontLeft.getCurrentPosition();
        lastFR_enc = frontRight.getCurrentPosition();
        lastBL_enc = backLeft.getCurrentPosition();
        lastBR_enc = backRight.getCurrentPosition();

        x = 0.0;
        y = 0.0;
        theta = 0.0;

//        imu.initialize(new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//        ));


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
    public void ChassisLocationTelemetry(){
        telemetry.addData("X Position (cm): ", x);
        telemetry.addData("Y Position (cm): ", y);
        telemetry.addData("Heading (rad): ", theta);


    }

    public void ChassisStop(){
        flPower=0;
        frPower=0;
        blPower=0;
        brPower=0;
        ChassisPowerTelemetry();
        setMotorPowers(0,0,0,0);
    }



    /**
     * 底盘定位，自解
     * 包含telemetry
     * Xposition
     * Yposition
     * Heading theta
     */
    public void Localization() {
        // 1. 获取当前时间和采样周期Δt（单位：s），避免积分误差
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.01) return; // 采样周期过小，跳过计算（防止除零/抖动）

        // 2. 获取当前编码器脉冲数，并计算脉冲增量ΔN（当前 - 上一次）
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        int deltaFL = currFL - lastFL_enc;
        int deltaFR = currFR - lastFR_enc;
        int deltaRL = currBL - lastBL_enc;
        int deltaRR = currBR - lastBR_enc;

        // 3. 脉冲增量 → 车轮转角增量Δθ（单位：rad）
        // 公式：Δθ = (2π * ΔN) / (PPR * 减速比)
        double deltaThetaFL = (2 * Math.PI * deltaFL) / (ENCODER_PPR * GEAR_RATIO);
        double deltaThetaFR = (2 * Math.PI * deltaFR) / (ENCODER_PPR * GEAR_RATIO);
        double deltaThetaRL = (2 * Math.PI * deltaRL) / (ENCODER_PPR * GEAR_RATIO);
        double deltaThetaRR = (2 * Math.PI * deltaRR) / (ENCODER_PPR * GEAR_RATIO);

        // 4. 车轮转角增量 → 车轮线位移增量Δs（单位：cm）
        // 公式：Δs = 车轮半径 * 转角增量
        double deltaSFL = WHEEL_RADIUS_CM * deltaThetaFL;
        double deltaSFR = WHEEL_RADIUS_CM * deltaThetaFR;
        double deltaSRL = WHEEL_RADIUS_CM * deltaThetaRL;
        double deltaSRR = WHEEL_RADIUS_CM * deltaThetaRR;

        // 5. 四轮位移 → 底盘瞬时全向速度（运动学正解，单位：cm/s，rad/s）
        // 核心正解公式，与控制逆解对应
        double vx = (deltaSFL + deltaSFR + deltaSRL + deltaSRR) / (4 * dt);
        double vy = (deltaSFL - deltaSFR - deltaSRL + deltaSRR) / (4 * dt);
        double omega = (-deltaSFL + deltaSFR - deltaSRL + deltaSRR) / (4 * D * dt);

        // 6. 瞬时速度 → 位姿增量（Δx, Δy, Δtheta），基于当前航向角theta解算
        double deltaTheta = omega * dt;
        double deltaX = (vx * Math.cos(theta) - vy * Math.sin(theta)) * dt;
        double deltaY = (vx * Math.sin(theta) + vy * Math.cos(theta)) * dt;

        // 7. 更新全局位姿（核心：累加增量）
        x += deltaX;
        y += deltaY;
        theta += deltaTheta;
//        theta=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // 航向角归一化（-π ~ π），防止角度无限增大
        theta = normalizeAngle(theta);



        // 8. 更新历史数据（为下一次采样做准备）
        lastTime = currentTime;
        lastFL_enc = currFL;
        lastFR_enc = currFR;
        lastBL_enc = currBL;
        lastBR_enc = currBR;
    }

    // -------------------------- 6. 工具方法：航向角归一化（-π ~ π） --------------------------
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
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
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }
    public void ChassisVelocityTelemetry(){
        telemetry.addData("Front Left Velocity (cm/s): ", frontLeft.getVelocity() * (Math.PI * WHEEL_RADIUS_CM * 2) / (ENCODER_PPR * GEAR_RATIO));
        telemetry.addData("Front Right Velocity (cm/s): ", frontRight.getVelocity() * (Math.PI * WHEEL_RADIUS_CM * 2) / (ENCODER_PPR * GEAR_RATIO));
        telemetry.addData("Back Left Velocity (cm/s): ", backLeft.getVelocity() * (Math.PI * WHEEL_RADIUS_CM * 2) / (ENCODER_PPR * GEAR_RATIO));
        telemetry.addData("Back Right Velocity (cm/s): ", backRight.getVelocity() * (Math.PI * WHEEL_RADIUS_CM * 2) / (ENCODER_PPR * GEAR_RATIO));
    }
    public void ChassisModeTelemetry(){
        telemetry.addData("NoHeadMode: ", NoHeadMode);
        telemetry.addData("movingspeed",DRIVE_SPEED);
        telemetry.addData("turnspeed",TURN_SPEED);

    }

}
