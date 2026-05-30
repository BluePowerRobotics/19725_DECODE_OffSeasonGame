package org.firstinspires.ftc.teamcode.OpModes.Actions_Near;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HypParams;

public class GoToShootingAreaAction_Near implements Action {
    private final Chassis chassis;
    private final Sweeper sweeper;
    private final boolean isRed;

    public GoToShootingAreaAction_Near(Chassis chassis, Sweeper sweeper, boolean isRed) {
        this.chassis = chassis;
        this.sweeper = sweeper;
        this.isRed = isRed;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        sweeper.setStop();
        sweeper.update();

        chassis.GoToNearShootingArea(isRed);

        ConvexPolygon targetArea = isRed ? HypParams.SHOOTING_AREA_NEAR_RED : HypParams.SHOOTING_AREA_NEAR_BLUE;
        if (HypParams.BoundingBox.inAbsolute(RobotPosition.getInstance().getPose2d()).IsIntersected(targetArea)) {
            chassis.stop();
            return false;
        }
        return true;
    }
}