package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

public class GoToShootingAreaAction implements Action {
    private final Chassis chassis;
    private final Sweeper sweeper;
    private boolean started = false;

    public GoToShootingAreaAction(Chassis chassis, Sweeper sweeper) {
        this.chassis = chassis;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        sweeper.setStop();
        sweeper.update();

        if (!started) {
            chassis.GoToShootingArea();
            started = true;
        }

        chassis.update();

        return !RobotPosition.getInstance().isAbleToShoot();
    }
}