package org.firstinspires.ftc.teamcode.Controllers.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.PIDController;

//单个弹射飞轮的PID控制器
@Config
public class Shooter {
    //被卡住的速度
    public static int SpeedTolerance = 35;
    //关闭的功率
    public static double BlockPower = -0.3;
    //找到motor
    public DcMotorEx shooterMotor;

    double[] speedBuffer = new double[10];
    Telemetry telemetry;
    //当前功率
    double Power = 0;
    //当前速度
    double current_speed = 0;
    //时间变量
    double current_time;
    double previous_time;
    //编码器变量
    double current_encoder = 0;
    //PID切换速度
    public static int PIDSwitchSpeed = 750;
    //PID参数(第一套
    public static double MinPower_1 = 0.1;//最小功率
    public static double k_p_1 = 0.01;
    public static double k_i_1 = 0.2;
    public static double k_d_1 = 0.05;
    public static double max_i_1 = 1;
    //PID参数(第二套
    public static double MinPower_2 = 0.3;
    public static double k_p_2 = 0.011;
    public static double k_d_2 = 0.08;
    public static double k_i_2 = 0.05;
    public static double max_i_2 = 0.6;
//加pid控制器
    private final PIDController pidController;

    //构造函数
    public Shooter(HardwareMap hardwareMap, Telemetry telemetryrc, String motorName, boolean ifReverse){
        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(ifReverse)
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.telemetry = telemetryrc;
        pidController = new PIDController(k_p_1, k_i_1, k_d_1, max_i_1);
        previous_time = System.currentTimeMillis();
    }

    /**
     * 射击函数，使用PID控制电机速度
     * @param targetSpeed 目标速度
     *                    两套PID间隔为去configure里面找
     *
     * @return 是否达到目标速度
     *
     */
    public boolean shoot(int targetSpeed){
        //根据速度选择PID参数
        if(targetSpeed <    PIDSwitchSpeed){
            pidController.setPID(k_p_1, k_i_1, k_d_1);
            pidController.setMaxI(max_i_1);
        }
        else{

            pidController.setPID(k_p_2, k_i_2, k_d_2);
            pidController.setMaxI(max_i_2);
        }
        //目标速度为0时，直接停止电机并重置PID
        if(targetSpeed == 0){
            shooterMotor.setPower(0);
            pidController.reset();
            previous_time = System.currentTimeMillis();
            return true;
        }
        //电机模式调整
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        current_time = System.currentTimeMillis();
        current_encoder = shooterMotor.getCurrentPosition();
        current_speed = shooterMotor.getVelocity(AngleUnit.DEGREES);
        double dt = (current_time - previous_time);
        if (dt <= 0) dt = 1; // 防止除零
        //计算PID
        Power = pidController.calculate(targetSpeed, current_speed, dt);
        //避免射球时出现负功率，导致震荡或电机损伤
        if(targetSpeed > 0 && targetSpeed < 750){
            Power = Range.clip(Power, MinPower_1, 1);
        }
        else if(targetSpeed > 0 && targetSpeed >=750){
            Power = Range.clip(Power, MinPower_2, 1);
        }
        shooterMotor.setPower(Power);
        previous_time = current_time;
        if(Math.abs(targetSpeed - current_speed) < SpeedTolerance){
            return true;
        } else {
            return false;
        }
    }
    public void block(){

        shooterMotor.setPower(BlockPower);
    }

    public double getCurrent_encoder(){
        return current_encoder;
    }
    public double getPower(){
        return Power;
    }
    public double getCurrent_speed(){
        return current_speed;
    }

}
