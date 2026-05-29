package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.utility.HypParams;

public class ShootAction implements Action {
    private final Turret turret;
    private final Sweeper sweeper;
    private int targetTagId;
    private final Chassis chassis;

    public ShootAction(Chassis chassis, Turret turret, int targetTagId, Sweeper sweeper) {
        this.chassis = chassis;
        this.turret = turret;
        this.targetTagId = targetTagId;
        this.sweeper = sweeper;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        turret.update(true, true, targetTagId);

        if (turret.getShootPhase() == Turret.ShootPhase.PREPARING) {
            if (!sweeper.isPreparing()) {
                sweeper.prepare(HypParams.PrepareAngle);
            }
        } else if (turret.getShootPhase() == Turret.ShootPhase.FIRING) {
            sweeper.setTrigger();
        } else if (turret.getShootPhase() == Turret.ShootPhase.IDLE && !sweeper.isPreparing()) {
            sweeper.setStop();
        }
        sweeper.update();
        chassis.stop();
        return !RobotPosition.getInstance().isEmpty();
    }
}