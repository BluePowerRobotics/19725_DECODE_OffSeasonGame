package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;

public class ShootAction implements Action {
    private final Turret turret;
    private final Sweeper sweeper;
    private int targetTagId;

    public ShootAction(Turret turret, int targetTagId, Sweeper sweeper) {
        this.turret = turret;
        this.targetTagId = targetTagId;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        sweeper.setStop();
        sweeper.update();

        turret.update(true, true, targetTagId);

        return !RobotPosition.getInstance().isEmpty();
    }
}