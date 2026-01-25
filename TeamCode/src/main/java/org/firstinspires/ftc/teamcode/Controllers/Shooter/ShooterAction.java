package org.firstinspires.ftc.teamcode.Controllers.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;
@Config
public class ShooterAction {
    MeanFilter FilterLeft;
    MeanFilter FilterRight;
    private Telemetry telemetry;
    private Shooter shooter;

    private boolean whetherReachTargetSpeed = false;

    public int WindowSize = 3;
    public ShooterAction(HardwareMap hardwareMap, Telemetry telerc) {
        telemetry = telerc;
        FilterLeft = new MeanFilter(WindowSize);
        shooter = new Shooter(hardwareMap, telemetry, "shooterMotor", true);

    }
    public boolean setShootSpeed(int TargetSpeedRC){
        boolean shootersituation;
        shooter.shoot(TargetSpeedRC);
        FilterLeft.filter(shooter.getCurrent_speed());
        shootersituation = (Math.abs(FilterLeft.getMean() - TargetSpeedRC) < Shooter.SpeedTolerance);
        whetherReachTargetSpeed = shootersituation;
        return (shootersituation);


    }
    public void setTelemetry(){
        telemetry.addData("Whether Reach Target Speed", this.whetherReachTargetSpeed());
        telemetry.addData("Shooter Power ", this.getPower());
        telemetry.addData("Shooter Speed ", this.getSpeed());
    }
    public boolean whetherReachTargetSpeed(){
        return whetherReachTargetSpeed;
    }
    public double getPower(){
        return shooter.Power;
    }
    public double getSpeed(){return shooter.getCurrent_speed();}

    }





