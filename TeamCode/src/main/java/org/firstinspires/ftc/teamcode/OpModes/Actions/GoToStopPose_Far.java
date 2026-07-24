package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret_2;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 停车Action：使用RoadRunner轨迹前往StopPose，同时将炮台复位到0度并停车
 */
public class GoToStopPose_Far implements Action {
    private final Action trajectoryAction;
    private final Turret_2 turret;

    public GoToStopPose_Far(MecanumDrive drive, Pose2d stopPose, Turret_2 turret) {
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(stopPose.position, stopPose.heading.toDouble())
                .build();
        this.turret = turret;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToStopPose", "Parking...");
        turret.reset();
        turret.rotate_to(0, 0);
        return trajectoryAction.run(packet);
    }
}