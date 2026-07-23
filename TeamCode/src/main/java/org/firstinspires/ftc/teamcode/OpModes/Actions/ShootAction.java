package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
@Config
//
public class ShootAction implements Action {
    private final Turret turret;
    private final Sweeper sweeper;
    private int targetTagId;
    private final Chassis chassis;
    private final long durationMs;
    private long startTimeMs = -1;

    public static int DEFAULT_DURATION_MS = 5000;

    public ShootAction(Chassis chassis, Turret turret, int targetTagId, Sweeper sweeper) {
        this(chassis, turret, targetTagId, sweeper, DEFAULT_DURATION_MS);
    }

    public ShootAction(Chassis chassis, Turret turret, int targetTagId, Sweeper sweeper, long durationMs) {
        this.chassis = chassis;
        this.turret = turret;
        this.targetTagId = targetTagId;
        this.sweeper = sweeper;
        this.durationMs = durationMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (startTimeMs < 0) {
            startTimeMs = System.currentTimeMillis();
        }

        RobotPosition.getInstance().update();

        if (turret.isLaunching()) {
            sweeper.setGiveArtifact();
        } else {
            sweeper.setStop();
        }
        sweeper.update();
        chassis.stop();
        turret.update(true, true, targetTagId);

        if (System.currentTimeMillis() - startTimeMs >= durationMs) {
            sweeper.setStop();
            turret.reset();
            return false;
        }
        return true;
    }
}
