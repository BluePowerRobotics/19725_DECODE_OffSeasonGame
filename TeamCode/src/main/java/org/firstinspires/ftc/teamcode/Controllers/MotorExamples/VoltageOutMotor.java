package org.firstinspires.ftc.teamcode.Controllers.MotorExamples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDSVAController;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.SlotConfig;


/**
 * ExampleVoltageOutMotor 类实现了使用电压输出的电机速度控制
 * 结合 PIDSVAController 实现高精度速度控制
 */
@Config
public class VoltageOutMotor {
    // Dashboard 热调参参数
    /** 比例系数 */
    public static double kP = 0.0;
    /** 积分系数 */
    public static double kI = 0.0;
    /** 微分系数 */
    public static double kD = 0.0;
    /** 积分上限 */
    public static double maxI = 1.0;
    /** 静态摩擦系数 */
    public static double kS = 2.3;
    /** 速度系数 */
    public static double kV = 0.0073;
    /** 加速度系数 */
    public static double kA = 0.0;
    /** 输出电压最小值 */
    public static double outputMin = -14.0;
    /** 输出电压最大值 */
    public static double outputMax = 14.0;

    /** 电机实例 */
    public final DcMotorEx motor;
    /** 电压输出控制器 */
    private final VoltageOut voltageOut;
    /** PIDSVAController 实例 */
    private final PIDSVAController controller;
    /** 控制器配置 */
    private final SlotConfig config;
    /** 遥测实例 */
    private final Telemetry telemetry;
    /** 目标速度 */
    private double targetVelocity = 0;
    /** 上次更新时间 */
    private long lastUpdateTime = 0;

    /** 达到速度的阈值 **/

    public double SpeedTolerance =20;

    public double currentVelocity=0;
    public double currentPower=0;

    /**
     * 构造函数，初始化电机和控制器
     * @param hardwareMap 硬件映射
     * @param motorName 电机名称
     * @param telemetry 遥测实例
     */
    public VoltageOutMotor(HardwareMap hardwareMap, String motorName, Telemetry telemetry, boolean ifReverse) {
        // 获取电机实例
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);


        // 配置电机
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        if(ifReverse)
            this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        else
            this.motor.setDirection(DcMotorEx.Direction.FORWARD);

        // 初始化电压输出控制器
        this.voltageOut = new VoltageOut(hardwareMap);

        // 初始化控制器配置
        this.config = new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        // 初始化PIDSVAController
        this.controller = new PIDSVAController().withSlot0(config);

        this.telemetry = telemetry;
    }

    public void setconfig(SlotConfig config){
        this.kP=config.kP;
        this.kI=config.kI;
        this.kD=config.kD;
        this.maxI=config.maxI;
        this.kS=config.kS;
        this.kV=config.kV;
        this.kA=config.kA;
        this.outputMin=config.outputMin;
        this.outputMax=config.outputMax;
    }

    /**
     * 设置目标速度
     * @param velocity 目标速度（单位：ticks per second）
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    /**
     * 更新电机控制
     * 1. 同步Dashboard参数
     * 2. 计算时间间隔
     * 3. 获取当前速度
     * 4. 计算控制器输出
     * 5. 转换为功率并设置给电机
     * 6. 输出遥测数据
     */
    public void update() {
        // 动态同步 SlotConfig 参数，支持 Dashboard 热调参
        config.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
              .withKS(kS).withKV(kV).withKA(kA)
              .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(config);
        
        // 计算时间间隔
        long now = System.currentTimeMillis();
        double dt = lastUpdateTime == 0 ? 0.02 : (now - lastUpdateTime) / 1000.0;
        lastUpdateTime = now;

        // 获取当前电机速度
        currentVelocity = motor.getVelocity();

        // 计算控制器输出电压
        double outputVoltage = controller.calculate(targetVelocity, currentVelocity, dt, true);

        // 将电压转换为功率
        double power = voltageOut.getVoltageOutPower(outputVoltage);

        // 设置电机功率
        motor.setPower(power);
        
        // 输出遥测数据
        this.currentPower=power;
        this.currentVelocity=motor.getVelocity();
//        telemetry.addData("TargetVelocity", targetVelocity);
//        telemetry.addData("CurrentVelocity", currentVelocity);
//        telemetry.addData("OutputVoltage", outputVoltage);
//        telemetry.addData("Power", power);
    }

    public boolean reachedTarget(){
        return (motor.getVelocity() - targetVelocity) <= SpeedTolerance;
    }

    /**
     * 停止电机
     */
    public void stop() {
        motor.setPower(0);
    }
    public double getVelocity(){return currentVelocity;}
    public double getPower(){return  currentPower;}
}
