package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

public class EatAction implements Action {
    private final Chassis chassis;
    private final Tracker tracker;
    private final Sweeper sweeper;

    public EatAction(Chassis chassis, Tracker tracker, Sweeper sweeper) {
        this.chassis = chassis;
        this.tracker = tracker;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        if (RobotPosition.getInstance().isFull()) {
            sweeper.setStop();
            sweeper.update();
            return false;
        }

        sweeper.setEat();
        sweeper.update();

        tracker.update();
        Tracker.Target bestTarget = tracker.getBestTarget();

        if (bestTarget != null) {
            double targetTheta = Math.atan2(bestTarget.centerY, bestTarget.centerX);
            chassis.GoTo(targetTheta);
        } else {
            return false;
        }

        return !RobotPosition.getInstance().isFull();
    }
}