package org.firstinspires.ftc.teamcode.Controllers.Turret.turner;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDSVAController;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.SlotConfig;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.VoltageOut;
import org.firstinspires.ftc.teamcode.utility.HypParams;

/**
 * TurretDegreeController 类实现炮台旋转控制
 * 支持 roll(水平旋转) 电机使用 PIDSVA 位置闭环控制
 * 支持 yaw(仰角) 舵机使用位置直接控制
 */
public class TurretDegreeController {

    public DcMotorEx rollMotor;
    private Servo yawServo;
    private VoltageOut voltageOut;
    private PIDSVAController controller;
    private SlotConfig config;
    private Telemetry telemetry;

    public static double ROLL_TICKS_PER_DEGREE = 1.187878787878;

    public static double SERVO_POSITION_PER_DEGREE = 0.0688816;// 这个值是根据舵机的实际测试测量得到的参数
    public static double YAW_OFFSET = 0.0; // yaw 初始角，用于校准舵机 0 度位置
    public static double ANGLE_TOLERANCE = 0.5;

    private double currentRoll = 0.0;
    private double currentYaw = 0.0;
    private double targetRoll = 0.0;
    private double targetYaw = 0.0;
    private long lastUpdateTime = 0;
    
    // 方向反转补偿相关变量
    private double lastTargetRoll = 0.0;          // 上一次的目标角度，用于检测方向变化
    private boolean isCompensating = false;       // 是否正在执行反转补偿
    private double compensationTarget = 0.0;      // 补偿目标角度
    private double originalTargetAfterCompensation = 0.0;  // 补偿完成后需要到达的原始目标

    public double kP = 1.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double maxI = 1.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double outputMin = -14.0;
    public double outputMax = 14.0;

    public static double YAW_SERVO_MIN = 0.0;
    public static double YAW_SERVO_MAX = 1.0;
    public static double YAW_ANGLE_MIN = 0.0;
    public static double YAW_ANGLE_MAX = 180.0;

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param rollMotorName 水平旋转电机名称
     * @param yawServoName 仰角舵机名称
     * @param telemetry 遥测实例
     */
    public TurretDegreeController(HardwareMap hardwareMap, String rollMotorName, String yawServoName, Telemetry telemetry) {
        this.rollMotor = hardwareMap.get(DcMotorEx.class, rollMotorName);
        this.yawServo = hardwareMap.get(Servo.class, yawServoName);
        this.telemetry = telemetry;

        initRollMotor();

        this.voltageOut = new VoltageOut(hardwareMap);

        this.config = new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        this.controller = new PIDSVAController().withSlot0(config);

        resetAngles();
    }

    /**
     * 默认构造函数，使用默认电机名称
     * @param hardwareMap 硬件映射
     */
    public TurretDegreeController(HardwareMap hardwareMap,Telemetry telemetry1) {
        this(hardwareMap, "roll", "yaw",telemetry1);
    }

    /**
     * 初始化水平旋转电机
     */
    private void initRollMotor() {
        rollMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rollMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rollMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rollMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    /**
     * 重置角度基准，将当前位置设为零点
     */
    public void resetAngles() {
        rollMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rollMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        currentRoll = 0.0;
        currentYaw = YAW_OFFSET;
        targetRoll = 0.0;
        targetYaw = YAW_OFFSET;
        yawServo.setPosition(0.0); // 初始位置对应 YAW_OFFSET 角度
        
        // 重置方向反转补偿状态
        lastTargetRoll = 0.0;
        isCompensating = false;
        compensationTarget = 0.0;
        originalTargetAfterCompensation = 0.0;
    }

    /**
     * 设置目标水平旋转角度
     * 当检测到方向反转时，会先额外转动 ReverseRollAngle 角度使齿轮重新卡死
     * @param roll 目标角度(度)
     */
    public void setTargetRoll(double roll) {
        double normalizedRoll = normalizeAngle(roll);
        
        // 如果正在执行补偿，先记录最终目标，等补偿完成后再执行
        if (isCompensating) {
            originalTargetAfterCompensation = normalizedRoll;
            return;
        }
        
        // 计算当前目标与上一次目标的差值（考虑角度归一化）
        double delta = normalizedRoll - lastTargetRoll;
        // 计算当前位置到新目标的方向
        double currentDelta = normalizedRoll - normalizeAngle(currentRoll);
        
        // 检测方向反转：新目标与上一次目标的方向相反（差值超过90度视为方向改变）
        // 且当前位置不在目标附近（避免在目标位置微调时误触发）
        boolean directionReversed = Math.abs(delta) > 90.0 
                && Math.abs(currentRoll - lastTargetRoll) > ANGLE_TOLERANCE * 2;
        
        if (directionReversed && HypParams.ReverseRollAngle > 0) {
            // 触发反转补偿：先向新方向多转 ReverseRollAngle 角度
            double compensationDirection = currentDelta > 0 ? 1 : -1;
            compensationTarget = normalizedRoll + compensationDirection * HypParams.ReverseRollAngle;
            compensationTarget = normalizeAngle(compensationTarget);
            originalTargetAfterCompensation = normalizedRoll;
            this.targetRoll = compensationTarget;
            isCompensating = true;
        } else {
            this.targetRoll = normalizedRoll;
        }
        
        lastTargetRoll = normalizedRoll;
    }

    /**
     * 设置目标仰角角度
     * @param yaw 目标角度(度)，范围[YAW_ANGLE_MIN, YAW_ANGLE_MAX]
     */
    public void setTargetYaw(double yaw) {
        this.targetYaw = Math.max(YAW_OFFSET, Math.min(YAW_ANGLE_MAX, yaw));
    }

    /**
     * 同时设置目标和仰角目标
     * @param roll 目标水平角度(度)
     * @param yaw 目标仰角(度)
     */
    public void setTarget(double roll, double yaw) {
        setTargetRoll(roll);
        setTargetYaw(yaw);
    }

    /**
     * 旋转到指定角度(需在主循环中持续调用)
     * @param roll 目标水平旋转角(度)
     * @param yaw 目标仰角(度)
     * @return 是否已到达目标位置
     */
    public boolean rotateTo(double roll, double yaw) {
        targetRoll = normalizeAngle(roll);
        targetYaw = yaw;

        boolean rollReached = rotateRollTo(targetRoll);
        boolean yawReached = rotateYawTo(targetYaw);

        return rollReached && yawReached;
    }

    /**
     * 旋转水平电机到目标角度 - PIDSVA位置闭环控制
     * @param targetAngle 目标水平角度(度)
     * @return 是否达到目标
     */
    private boolean rotateRollTo(double targetAngle) {
        setTargetRoll(targetAngle);
        currentRoll = rollMotor.getCurrentPosition() / ROLL_TICKS_PER_DEGREE;
        currentRoll = normalizeAngle(currentRoll);
        return Math.abs(currentRoll - targetAngle) <= ANGLE_TOLERANCE;
    }

    /**
     * 旋转仰角舵机到目标角度 - 舵机位置直接控制
     * @param targetAngle 目标仰角(度)
     * @return 是否达到目标
     */
    private boolean rotateYawTo(double targetAngle) {
        setTargetYaw(targetAngle);
        currentYaw = yawServo.getPosition() / SERVO_POSITION_PER_DEGREE + YAW_OFFSET;
        return Math.abs(currentYaw - targetAngle) <= ANGLE_TOLERANCE;
    }

    /**
     * 每帧调用的总更新函数
     * 包含所有控制逻辑和遥测更新
     */
    public void update() {
        config.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
              .withKS(kS).withKV(kV).withKA(kA)
              .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(config);

        long now = System.currentTimeMillis();
        double dt = lastUpdateTime == 0 ? 0.02 : (now - lastUpdateTime) / 1000.0;
        lastUpdateTime = now;
        
        //这里传进去的roll和yaw一定是原始定义，可能要转换才能输给舵机
        currentRoll = rollMotor.getCurrentPosition() / ROLL_TICKS_PER_DEGREE;
        currentRoll = normalizeAngle(currentRoll);

        // 检查反转补偿是否完成
        if (isCompensating) {
            double currentCompensationError = Math.abs(normalizeAngle(currentRoll - compensationTarget));
            if (currentCompensationError <= ANGLE_TOLERANCE) {
                // 补偿完成，切换到原始目标
                targetRoll = originalTargetAfterCompensation;
                isCompensating = false;
            }
        }
        
        double outputVoltage = controller.calculate(targetRoll, currentRoll, dt, false);
        double power = voltageOut.getVoltageOutPower(outputVoltage);
        rollMotor.setPower(power);

        double yawDelta = targetYaw - YAW_OFFSET; // 计算相对于初始角的变化量
        double yawTargetServo = yawDelta * SERVO_POSITION_PER_DEGREE;
        yawServo.setPosition(Math.max(YAW_SERVO_MIN, Math.min(YAW_SERVO_MAX, yawTargetServo)));
        currentYaw = yawServo.getPosition() / SERVO_POSITION_PER_DEGREE + YAW_OFFSET;

        if (telemetry != null) {
            telemetry.addData("TargetRoll", targetRoll);
            telemetry.addData("CurrentRoll", currentRoll);
            telemetry.addData("TargetYaw", targetYaw);
            telemetry.addData("CurrentYaw", currentYaw);
            telemetry.addData("RollPower", power);
            telemetry.addData("IsCompensating", isCompensating);
        }
    }

    /**
     * 检查是否到达目标位置
     * @return 是否两个轴都已到达目标
     */
    public boolean reachedTarget() {
        double currentNormalized = normalizeAngle(rollMotor.getCurrentPosition() / ROLL_TICKS_PER_DEGREE);
        return Math.abs(currentNormalized - targetRoll) <= ANGLE_TOLERANCE
                && Math.abs(currentYaw - targetYaw) <= ANGLE_TOLERANCE;
    }

    /**
     * 停止电机
     */
    public void stop() {
        rollMotor.setPower(0);
    }

    /**
     * 获取当前角度
     * @return 包含[roll, yaw]的数组
     */
    public double[] get_angle() {
        return new double[]{currentRoll, currentYaw};
    }

    /**
     * 获取目标角度
     * @return 包含[targetRoll, targetYaw]的数组
     */
    public double[] getTarget() {
        return new double[]{targetRoll, targetYaw};
    }

    /**
     * 获取当前瞄准误差
     * @return 包含[rollError, yawError]的数组
     */
    public double[] getAimError() {
        double rollError = Math.abs(normalizeAngle(targetRoll - currentRoll));
        double yawError = Math.abs(targetYaw - currentYaw);
        return new double[]{rollError, yawError};
    }

    /**
     * 角度归一化到[-180, 180]度范围
     * @param angle 原始角度(度)
     * @return 归一化后的角度(度)
     */
    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle > 180.0) {
            angle -= 360.0;
        } else if (angle < -180.0) {
            angle += 360.0;
        }
        return angle;
    }
}
