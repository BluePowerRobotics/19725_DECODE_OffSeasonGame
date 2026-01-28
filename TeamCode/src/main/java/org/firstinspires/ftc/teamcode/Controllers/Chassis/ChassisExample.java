package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FTC 麦克纳姆轮全向控制 + 编码器定位 样例程序
 * TeleOp 模式，手柄遥控+实时定位显示
 * 硬件：REV Expansion Hub + 4路带编码器直流电机 + 游戏手柄（FTC 标准）
 */
@TeleOp(name = "MecanumDrive_EncoderLocalization", group = "FTC")
public class ChassisExample extends OpMode {

    // -------------------------- 1. 硬件声明 --------------------------
    private DcMotorEx frontLeft, frontRight, backLeft, backRight; // 带编码器的电机对象
    private ElapsedTime runtime = new ElapsedTime(); // 计时工具，用于计算采样周期

    // -------------------------- 2. 核心参数标定区（根据实际硬件修改！） --------------------------
    // 电机端口配置（对应 Expansion Hub 端口，需与机器人配置一致）
    private static final String FL_MOTOR = "fL";  // 前左电机端口名
    private static final String FR_MOTOR = "fR"; // 前右电机端口名
    private static final String RL_MOTOR = "bL";   // 后左电机端口名
    private static final String RR_MOTOR = "bR";  // 后右电机端口名

    // 机械参数（必须精准测量/标定，定位和控制的核心）
    private static final double WHEEL_RADIUS_CM = 5.0;    // 麦克纳姆轮有效半径，单位：cm
    private static final double L = 15.0;                 // 底盘前后轮中心纵向距离，单位：cm
    private static final double W = 15.0;                 // 底盘左右轮中心横向距离，单位：cm
    private static final double D = L + W;                // 复合参数，无需修改
    private static final double GEAR_RATIO = 1.0;         // 减速比（无减速箱则为1，有则填实际值，如50:1则为50）
    private static final int ENCODER_PPR = 1024;          // 电机编码器线数（PPR），正交输出可×4（如1024×4=4096）

    // 控制参数
    private static final double DRIVE_SPEED = 0.8;        // 最大驱动速度（0~1，防止打滑）
    private static final double TURN_SPEED = 0.6;         // 最大旋转速度（0~1）
    private static final double DEADZONE = 0.1;           // 手柄死区（消除摇杆漂移，0.05~0.2）

    // 定位参数
    private double lastTime = 0.0;                       // 上一次采样时间，单位：s
    // 底盘全局位姿（x:X轴坐标 cm，y:Y轴坐标 cm，theta:航向角 rad，初始为原点）
    private double x = 0.0, y = 0.0, theta = 0.0;
    // 上一次编码器脉冲数（用于计算脉冲增量）
    private int lastFLenc = 0, lastFRenc = 0, lastRLenc = 0, lastRRenc = 0;

    // -------------------------- 3. 初始化方法（run once when init pressed） --------------------------
    @Override
    public void init() {
        // 初始化电机硬件（与机器人配置端口名一致）
        frontLeft = hardwareMap.get(DcMotorEx.class, FL_MOTOR);
        frontRight = hardwareMap.get(DcMotorEx.class, FR_MOTOR);
        backLeft = hardwareMap.get(DcMotorEx.class, RL_MOTOR);
        backRight = hardwareMap.get(DcMotorEx.class, RR_MOTOR);

        // 电机转向配置（关键！根据实际接线标定，反转用REVERSE，正转用FORWARD）
        // 右斜麦克纳姆轮标准配置：前右/后右电机反转，前左/后左正转（可根据实际运动调整）
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // 编码器模式配置：重置并设置为增量模式
        resetEncoders();
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 关闭电机制动（遥控模式更顺滑，定位模式可开启）
        setMotorBrake(false);

        // 初始化计时和编码器初始值
        runtime.reset();
        lastTime = runtime.seconds();
        lastFLenc = frontLeft.getCurrentPosition();
        lastFRenc = frontRight.getCurrentPosition();
        lastRLenc = backLeft.getCurrentPosition();
        lastRRenc = backRight.getCurrentPosition();

        // 调试信息：初始化完成
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Calibration", "Check wheel radius/gear ratio/encoder PPR");
        telemetry.update();
    }

    // -------------------------- 4. 循环方法（run continuously after start pressed） --------------------------
    @Override
    public void loop() {
        // 步骤1：手柄输入处理（左摇杆：全向移动，右摇杆X：旋转）
        double driveX = -gamepad1.left_stick_x * DRIVE_SPEED;  // 左/右横移（FTC 手柄X轴默认反向，取负修正）
        double driveY = -gamepad1.left_stick_y * DRIVE_SPEED;  // 前/后移动（FTC 手柄Y轴默认反向，取负修正）
        double turn = gamepad1.right_stick_x * TURN_SPEED;     // 旋转（右摇杆X轴）
        // 手柄死区处理：小于死区则置0，防止漂移
        driveX = Math.abs(driveX) < DEADZONE ? 0 : driveX;
        driveY = Math.abs(driveY) < DEADZONE ? 0 : driveY;
        turn = Math.abs(turn) < DEADZONE ? 0 : turn;

        // 步骤2：麦克纳姆轮运动学逆解 → 计算四轮目标功率
        // 核心公式：ω = (v_x ± v_y ± D·ω_theta) / r → 转换为功率（-1~1）
        double flPower = driveY + driveX - turn * D / WHEEL_RADIUS_CM;
        double frPower = driveY - driveX + turn * D / WHEEL_RADIUS_CM;
        double rlPower = driveY - driveX - turn * D / WHEEL_RADIUS_CM;
        double rrPower = driveY + driveX + turn * D / WHEEL_RADIUS_CM;

        // 步骤3：功率归一化（防止超过电机最大功率(-1~1)导致堵转）
        double maxPower = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(rlPower), Math.abs(rrPower)))));
        flPower /= maxPower;
        frPower /= maxPower;
        rlPower /= maxPower;
        rrPower /= maxPower;

        // 步骤4：设置电机功率，执行运动
        setMotorPowers(flPower, frPower, rlPower, rrPower);

        // 步骤5：编码器定位核心逻辑 → 实时解算底盘位姿
        updateLocalization();

        // 步骤6：实时调试信息显示（手机/电脑端 FTC Driver Station 查看）
        telemetry.addData("Status", "Running (Time: %.1fs)", runtime.seconds());
        telemetry.addData("Motor Powers", "FL: %.2f | FR: %.2f | RL: %.2f | RR: %.2f", flPower, frPower, rlPower, rrPower);
        telemetry.addLine("--- 编码器定位数据 ---");
        telemetry.addData("Global Pose", "X: %.1fcm | Y: %.1fcm | Theta: %.2frad (%.0f°)",
                x, y, theta, Math.toDegrees(theta)); // 同时显示弧度和角度，更直观
        telemetry.addData("Encoder Counts", "FL: %d | FR: %d | RL: %d | RR: %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.update();
    }

    // -------------------------- 5. 核心方法：编码器定位更新（航位推算） --------------------------
    private void updateLocalization() {
        // 1. 获取当前时间和采样周期Δt（单位：s），避免积分误差
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.01) return; // 采样周期过小，跳过计算（防止除零/抖动）

        // 2. 获取当前编码器脉冲数，并计算脉冲增量ΔN（当前 - 上一次）
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currRL = backLeft.getCurrentPosition();
        int currRR = backRight.getCurrentPosition();

        int deltaFL = currFL - lastFLenc;
        int deltaFR = currFR - lastFRenc;
        int deltaRL = currRL - lastRLenc;
        int deltaRR = currRR - lastRRenc;

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

        // 7. 更新全
        // 局位姿（核心：累加增量）
        x += deltaX;
        y += deltaY;
        theta += deltaTheta;
        // 航向角归一化（-π ~ π），防止角度无限增大
        theta = normalizeAngle(theta);

        // 8. 更新历史数据（为下一次采样做准备）
        lastTime = currentTime;
        lastFLenc = currFL;
        lastFRenc = currFR;
        lastRLenc = currRL;
        lastRRenc = currRR;
    }

    // -------------------------- 6. 工具方法：航向角归一化（-π ~ π） --------------------------
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // -------------------------- 7. 工具方法：设置四轮电机功率 --------------------------
    private void setMotorPowers(double fl, double fr, double rl, double rr) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(rl);
        backRight.setPower(rr);
    }

    // -------------------------- 8. 工具方法：重置编码器计数 --------------------------
    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // -------------------------- 9. 工具方法：设置电机制动模式 --------------------------
    private void setMotorBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    // -------------------------- 10. 停止方法（run once when stop pressed） --------------------------
    @Override
        public void stop() {
            // 停止所有电机，开启制动
            setMotorPowers(0, 0, 0, 0);
            setMotorBrake(true);
            // 显示停止信息
            telemetry.addData("Status", "Stopped");
            telemetry.addData("Final Pose", "X: %.1fcm | Y: %.1fcm | Theta: %.0f°",
                    x, y, Math.toDegrees(theta));
            telemetry.update();
        }
}