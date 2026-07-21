package org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDSVAController;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.SlotConfig;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.VoltageOut;

@Config
public class Shooter {
    /** 比例系数 */
    public static double kP = 0.08;
    /** 积分系数 */
    public static double kI = 0;
    /** 微分系数 */
    public static double kD = 0;
    /** 积分上限 */
    public static double maxI = 1.0;
    /** 静态摩擦系数 */
    public static double kS = 0.8;
    /** 速度系数 */
    public static double kV = 0.0049;
    /** 加速度系数 */
    public static double kA = 0.0;
    /** 输出电压最小值 */
    public static double outputMin = -14.0;
    /** 输出电压最大值 */
    public static double outputMax = 14.0;

    public static double VelocityTolerance = 20;
    /** 电机实例 */
    public  final DcMotorEx shooterL;
    private final DcMotorEx shooterR;
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

    private double currentVelocity =0;
    private double outputVoltage=0;
    private double power=0;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc){
        this.shooterL = hardwareMap.get(DcMotorEx.class, "motorL");
        this.shooterR = hardwareMap.get(DcMotorEx.class, "motorR");


        // 配置电机
        this.shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.shooterR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        this.shooterL.setDirection(DcMotorEx.Direction.FORWARD);

        this.shooterR.setDirection(DcMotorEx.Direction.REVERSE);

        // 初始化电压输出控制器
        this.voltageOut = new VoltageOut(hardwareMap);

        // 初始化控制器配置
        this.config = new SlotConfig()
                .withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        // 初始化PIDSVAController
        this.controller = new PIDSVAController().withSlot0(config);

        this.telemetry = telemetryrc;

    }
    public boolean setTargetVelocity(int targetVelocity){
        this.targetVelocity =targetVelocity;
        return reachedVelocity();
    }
    public boolean reachedVelocity(){
        return (Math.abs(currentVelocity - targetVelocity) < VelocityTolerance);//&&shooterR.reachedTarget();
    }

    public void update(){
        if(targetVelocity == 0){
            //绕过计算直接停电机。更加保险
            shooterL.setPower(0);
            shooterR.setPower(0);

            return;
        }
        //两套PID是不对的，当时是因为没有速度前馈项，所以强行用了两套PID参数，一套针对低速，一套针对高速。
        // 现在有了速度前馈项，理论上不需要两套了，所以统一。
        config.withKP(kP).withKI(kI).withKD(kD).withMaxI(maxI)
                .withKS(kS).withKV(kV).withKA(kA)
                .withOutputLimits(outputMin, outputMax);
        controller.resetSlot(config);

        long now = System.currentTimeMillis();
        double dt = lastUpdateTime == 0 ? 0.02 : (now - lastUpdateTime) / 1000.0;
        lastUpdateTime = now;
        currentVelocity = shooterL.getVelocity();
        outputVoltage = controller.calculate(targetVelocity, currentVelocity, dt, true);
        power = voltageOut.getVoltageOutPower(outputVoltage);

        shooterL.setPower(power);
        shooterR.setPower(power);

        telemetry.addData("TargetVelocity", targetVelocity);
        telemetry.addData("CurrentVelocity", currentVelocity);
        telemetry.addData("OutputVoltage", outputVoltage);
        telemetry.addData("Power", power);
    }
    public double getPowerL(){return shooterL.getPower();}
    public double getPowerR(){return shooterR.getPower();}
    public double getSpeedL(){return shooterL.getVelocity();}
    public double getSpeedR(){return shooterR.getVelocity();}
    public void setTelemetry(){
        /*
        telemetry.addData("Shooter PowerL*1000", getPowerL()*1000);
        telemetry.addData("Shooter PowerR*1000", getPowerR()*1000);
        telemetry.addData("Shooter Target Speed", this.targetVelocity);
        telemetry.addData("Shooter VelocityL", getSpeedL());
        telemetry.addData("Shooter VelocityR", getSpeedR());

         */
    }

    public double getCurrentVelocity(){
        return currentVelocity;
    }
}
