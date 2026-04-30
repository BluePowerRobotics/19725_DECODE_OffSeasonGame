package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;

public class StopAction implements Action {
    private final Chassis chassis;

    public StopAction(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        chassis.stop();
        return false;
    }
}