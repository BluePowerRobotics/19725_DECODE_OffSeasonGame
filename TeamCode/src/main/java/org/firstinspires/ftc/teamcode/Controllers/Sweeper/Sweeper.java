package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Sweeper {
    //TODO 改成速度闭环/基于电压输出的开环
    public DcMotorEx motor;

    private Telemetry telemetry;
    

    public static int EatVel = 1960;
    public static int GiveTheArtifactVel = 1960;
    public static int OutputVel = -960;
    
    private int targetVelocity = 0;
    
    public static int ForR = 0;
    
    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        setDirection();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    private void setDirection() {
        switch(ForR) {
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }
    
    public void setEat() {
        targetVelocity = EatVel;
    }
    
    public void setGiveArtifact() {
        targetVelocity = GiveTheArtifactVel;
    }
    
    public void setOutput() {
        targetVelocity = OutputVel;
    }
    
    public void setStop() {
        targetVelocity = 0;
    }
    
    public void setTargetVelocity(int velocity) {
        targetVelocity = velocity;
    }
    
    public void setPower(double power) {
        motor.setPower(power);
    }
    
    public void update() {
        motor.setVelocity(targetVelocity);
    }
    
    public double getPower() {
        return motor.getPower();
    }
    
    public double getVel() {
        return motor.getVelocity();
    }
    
    public int getFR() {
        return ForR;
    }
    
    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    
    public void setTelemetry() {
        telemetry.addData("Sweeper Velocity", getVel());
        telemetry.addData("Sweeper Power*1000", getPower()*1000);
        telemetry.addData("Sweeper Current", getCurrent());
    }
}
