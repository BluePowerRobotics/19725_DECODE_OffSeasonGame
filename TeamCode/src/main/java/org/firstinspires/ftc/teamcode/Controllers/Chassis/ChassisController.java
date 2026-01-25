package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChassisController {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private ElapsedTime runtime = new ElapsedTime(); // 计时工具，用于计算采样周期

    HardwareMap hardwareMap;
    Telemetry telemetry;

    // 机械参数
    private static final double WHEEL_RADIUS_CM = 5.0;    // 麦克纳姆轮有效半径，单位：cm
    private static final double L = 15.0;                 // 底盘前后轮中心纵向距离，单位：cm
    private static final double W = 15.0;                 // 底盘左右轮中心横向距离，单位：cm
    private static final double D = L + W;                // 复合参数，无需修改
    private static final double GEAR_RATIO = 1.0;         // 减速比（无减速箱则为1，有则填实际值，如50:1则为50）
    private static final int ENCODER_PPR = 1024;          // 电机编码器线数（PPR），正交输出可×4（如1024×4=4096）


    // 电机端口配置
    private static final String FL_MOTOR = "frontLeft";  // 前左电机端口名
    private static final String FR_MOTOR = "frontRight"; // 前右电机端口名
    private static final String RL_MOTOR = "backLeft";   // 后左电机端口名
    private static final String BR_MOTOR = "backRight";  // 后右电机端口名

    public int lastFLenc = 0, lastFRenc = 0, lastBLenc = 0, lastBRenc = 0;


    // 定位参数
    public double lastTime = 0.0;                       // 上一次采样时间，单位：s
    // 底盘全局位姿（x:X轴坐标 cm，y:Y轴坐标 cm，theta:航向角 rad，初始为原点）
    public double x = 0.0, y = 0.0, theta = 0.0;

    public ChassisController(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotorEx.class, FL_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, FR_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, RL_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, BR_MOTOR);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

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



        // 初始化计时和编码器初始值
        runtime.reset();
        lastTime = runtime.seconds();
        lastFLenc = frontLeft.getCurrentPosition();
        lastFRenc = frontRight.getCurrentPosition();
        lastBLenc = backLeft.getCurrentPosition();
        lastBRenc = backRight.getCurrentPosition();


    }

    public void ChassisMoving(){

    }
    public void ChassisTelemetry(double x, double y ,double theta){
        telemetry.addData("X Position (cm): ", x);
        telemetry.addData("Y Position (cm): ", y);
        telemetry.addData("Heading (rad): ", theta);


    }
    private void Localization() {
        // 1. 获取当前时间和采样周期Δt（单位：s），避免积分误差
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.01) return; // 采样周期过小，跳过计算（防止除零/抖动）

        // 2. 获取当前编码器脉冲数，并计算脉冲增量ΔN（当前 - 上一次）
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        int deltaFL = currFL - lastFLenc;
        int deltaFR = currFR - lastFRenc;
        int deltaRL = currBL - lastBLenc;
        int deltaRR = currBR - lastBRenc;

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
        // 航向角归一化（-π ~ π），防止角度无限增大
        theta = normalizeAngle(theta);

        // 8. 更新历史数据（为下一次采样做准备）
        lastTime = currentTime;
        lastFLenc = currFL;
        lastFRenc = currFR;
        lastBLenc = currBL;
        lastBRenc = currBR;
    }

    // -------------------------- 6. 工具方法：航向角归一化（-π ~ π） --------------------------
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
