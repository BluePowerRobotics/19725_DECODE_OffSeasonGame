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
    private Shooter shooter_Left;
    private Shooter shooter_Right;

    private boolean whetherReachTargetSpeed = false;

    public static int WindowSize = 3;
    public ShooterAction(HardwareMap hardwareMap, Telemetry telerc) {
        telemetry = telerc;
        FilterLeft = new MeanFilter(WindowSize);
        FilterRight = new MeanFilter(WindowSize);
        shooter_Left = new Shooter(hardwareMap, telemetry, "shooterMotor1", true);
        shooter_Right = new Shooter(hardwareMap, telemetry, "shooterMotor2", false);

    }
    public boolean setShootSpeed(int TargetSpeedRC){
        boolean left;
        shooter_Left.shoot(TargetSpeedRC);
        boolean right;
        shooter_Right.shoot(TargetSpeedRC);
        FilterLeft.filter(shooter_Left.getCurrent_speed());
        FilterRight.filter(shooter_Right.getCurrent_speed());
        left = (Math.abs(FilterLeft.getMean() - TargetSpeedRC) < Shooter.SpeedTolerance);
        right = (Math.abs(FilterRight.getMean() - TargetSpeedRC) < Shooter.SpeedTolerance);
        whetherReachTargetSpeed = left && right;
        //todo 检查这里所有&&和||的逻辑
        return (left && right);

    }
    public void setTelemetry(){
        telemetry.addData("Whether Reach Target Speed", this.whetherReachTargetSpeed());
        telemetry.addData("Shooter Power Left", this.getPowerLeft());
        telemetry.addData("Shooter Power Right", this.getPowerRight());
        telemetry.addData("Shooter Speed Left", this.getCurrent_speed_Left());
        telemetry.addData("Shooter Speed Right", this.getCurrent_speed_Right());

    }
    public boolean whetherReachTargetSpeed(){
        return whetherReachTargetSpeed;
    }
    public double getPowerLeft(){
        return shooter_Left.Power;
    }
    public double getPowerRight(){
        return shooter_Right.Power;
    }

    public double getCurrent_speed_Left(){
        return shooter_Left.current_speed;
    }
    public double getCurrent_speed_Right(){
        return shooter_Right.current_speed;
    }
}
