package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class Sweeper {
    public DcMotorEx motor;
    public static int EatVel = 1960;
    public static int GiveTheArtifactVel = 1960;
    public static int OutputVel = -960;

    public static int ForR=0;

    /**
     *
     * @param hardwareMap 你抄一遍吧。
     */
    public Sweeper(HardwareMap hardwareMap){
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        switch(ForR){
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     *
     * @param power double -1.0~1.0
     *
     * @return void
     */
    public void Eat(double power){
        motor.setPower(power);
    }

    /**
     * 这玩意按设定功率行动
     */
    public void Eat(){
        motor.setVelocity(EatVel);
    }

    /**
     * 吐球 power看调参
     */
    public void GiveArtifact(){
        motor.setPower(GiveTheArtifactVel);
    }

    /**
     * 停止
     */
    public void stop(){motor.setVelocity(0);}
    /**
     * 吐出
     */
    public void output(){motor.setVelocity(OutputVel);}
    /**
     *
     * @return 当前电机功率
     */
    public double getPower(){
        return motor.getPower();
    }
    /**
     *
     * @return 当前电机速度
     */
    public double getVel(){return motor.getVelocity();}
    /**
     * @return  0为reverse；1为forward
     */
    public int getFR(){return ForR;}
    /**
     *
     * @return 当前电机电流
     */
    public double getCurrent(){return motor.getCurrent(CurrentUnit.AMPS);}



}
