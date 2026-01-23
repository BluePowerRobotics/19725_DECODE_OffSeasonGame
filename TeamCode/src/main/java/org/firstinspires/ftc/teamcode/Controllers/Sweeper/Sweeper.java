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
    public void Eat(double power){
        motor.setPower(power);
    }
    public void Eat(){
        motor.setVelocity(EatVel);
    }
    public void GiveArtifact(){
        motor.setPower(GiveTheArtifactVel);
    }
    public void stop(){motor.setVelocity(0);}
    public void output(){motor.setVelocity(OutputVel);}
    public double getPower(){
        return motor.getPower();
    }
    public double getVel(){return motor.getVelocity();}
    /**
     * @return  0为reverse；1为forward
     */
    public int getFR(){return ForR;}
    public double getCurrent(){return motor.getCurrent(CurrentUnit.AMPS);}



}
