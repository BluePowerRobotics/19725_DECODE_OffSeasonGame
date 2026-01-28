package org.firstinspires.ftc.teamcode.Controllers.Turret.turning;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.TurretAimingCalculator;

public class TurretAimController {
    // 核心组件
    private TurretAimingCalculator aimingCalculator; // 瞄准角度计算器
    private DcMotorEx turretMotor;                   // 炮台驱动电机（使用DcMotorEx提升精度）
    private ElapsedTime runtime = new ElapsedTime(); // 时间戳（用于电机控制稳定性）

    public static double MOTOR_TICKS_PER_REV = 28;   // REV Hex Motor每转脉冲数（根据实际电机修改）
    // 电机参数（无减速比，适配REV Hex Motor等常见FTC电机）
    public static double TICKS_PER_DEGREE = MOTOR_TICKS_PER_REV / 360.0; // 每度对应的编码器脉冲数
    public static double MOTOR_MAX_POWER = 0.8;      // 电机最大功率（避免转速过快）
    public static double ANGLE_TOLERANCE = 0.5;      // 瞄准容差（度），小于该值则停止电机

    // 状态变量
    private double currentTurretAngle = 0.0; // 炮台当前相对机器人的朝向角（度，逆时针为正）
    private double targetTurretAngle = 0.0;  // 炮台需要到达的目标朝向角（度）

    /**
     * 构造方法：初始化炮台控制器
     * @param hardwareMap FTC硬件映射对象
     * @param turretMotorName 炮台电机在配置文件中的名称

     */
    public TurretAimController(HardwareMap hardwareMap, String turretMotorName){
        // 初始化瞄准计算器
        aimingCalculator = new TurretAimingCalculator(0.0, 0.0); // 初始目标位置可后续更新


        // 初始化炮台电机
        turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
        initTurretMotor();

        // 初始化炮台角度（以电机当前编码器位置为0度基准）
        resetTurretAngle();
    }

    /**
     * 初始化电机参数（关键：无减速比，编码器模式，方向校准）
     */
    private void initTurretMotor() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // 重置编码器
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);      // 启用编码器反馈
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // 断电刹车
        // 电机方向校准：根据实际硬件调整（true/false），确保计算的顺时针旋转对应电机正转
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    /**
     * 设置瞄准目标位置（世界坐标系）
     * @param targetX 目标X坐标
     * @param targetY 目标Y坐标
     */
    public void setTargetPosition(double targetX, double targetY) {
        aimingCalculator.setTargetPosition(targetX, targetY);
    }

    /**
     * 重置炮台角度基准（将当前电机位置设为0度）
     */
    public void resetTurretAngle() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        currentTurretAngle = 0.0;
        targetTurretAngle = 0.0;
    }

    /**
     * 核心更新函数（主程序每一帧调用）
     * @param robotX 机器人中心世界坐标系X坐标
     * @param robotY 机器人中心世界坐标系Y坐标
     * @param robotTheta 机器人方向角（度，逆时针为正，前进方向与X轴夹角）
     * @return 当前炮台相对机器人的朝向角（度，逆时针为正）
     */
    public double update(double robotX, double robotY, double robotTheta) {
        // 1. 计算炮台需要旋转的角度（delta_theta：顺时针为正，逆时针为负）
        double deltaTheta = aimingCalculator.calculateTurretRotateAngle(
                robotX, robotY, robotTheta, currentTurretAngle
        );

        // 2. 计算炮台目标朝向角（相对机器人）
        targetTurretAngle = currentTurretAngle - deltaTheta; // 转换为逆时针正方向的目标角度
        targetTurretAngle = normalizeAngle(targetTurretAngle); // 归一化到[-180, 180]度

        // 3. 控制电机旋转到目标角度
        ToTargetAngle(targetTurretAngle);

        // 4. 更新当前炮台角度（从编码器读取实际值，保证精度）
        currentTurretAngle = getTurretAngle();
        currentTurretAngle = normalizeAngle(currentTurretAngle);

        // 5. 返回当前炮台朝向角
        return currentTurretAngle;
    }

    /**
     * 控制电机旋转到目标角度
     * @param targetAngle 炮台目标朝向角（度，相对机器人，逆时针为正）
     */
    private void ToTargetAngle(double targetAngle) {
        // 计算目标编码器位置
        int targetTicks = (int) (targetAngle * TICKS_PER_DEGREE);
        // 计算当前编码器位置
        int currentTicks = turretMotor.getCurrentPosition();
        // 计算角度误差
        double angleError = Math.abs(targetAngle - getTurretAngle());

        // 误差小于容差时停止电机，否则以可控功率驱动
        if (angleError < ANGLE_TOLERANCE) {
            turretMotor.setPower(0.0);
        } else {
            // 简单比例控制（P控制），避免电机急停/急启
            double power = Math.min(MOTOR_MAX_POWER, Math.max(-MOTOR_MAX_POWER, angleError / 90.0 * MOTOR_MAX_POWER));
            // 调整功率方向：根据目标位置与当前位置的差值
            power = (targetTicks > currentTicks) ? power : -power;
            turretMotor.setPower(power);

            // 可选：高精度模式（使用RUN_TO_POSITION），适合静态瞄准
            // turretMotor.setTargetPosition(targetTicks);
            // turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            // turretMotor.setPower(power);
        }
    }

    /**
     * 从电机编码器读取当前炮台角度
     * @return 炮台相对机器人的朝向角（度，逆时针为正）
     */
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    /**
     * 角度归一化到[-180, 180]度范围
     * @param angle 原始角度（度）
     * @return 归一化后的角度（度）
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

    /**
     * 获取当前瞄准误差（度）
     * @return 炮台当前朝向与目标朝向的夹角（度）
     */
    public double getAimError() {
        return Math.abs(normalizeAngle(targetTurretAngle - currentTurretAngle));
    }
}
