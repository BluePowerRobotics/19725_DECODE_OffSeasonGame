package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;

public class ShootActionManual implements Action {
    private final Turret turret;
    private final Sweeper sweeper;
    private int targetTagId;
    private final Chassis chassis;

    public double roll= 0;
    public double yaw = 45;
    public int speed = 2000;
    public ShootActionManual(Chassis chassis, Turret turret, int targetTagId, Sweeper sweeper) {
        this.chassis = chassis;
        this.turret = turret;
        this.targetTagId = targetTagId;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        if (turret.isLaunching()) {
            sweeper.setGiveArtifact();
        } else {
            sweeper.setStop();
        }
        sweeper.update();
        chassis.stop();
        turret.update(roll, yaw, speed, true);
        return !RobotPosition.getInstance().isEmpty();
    }
}