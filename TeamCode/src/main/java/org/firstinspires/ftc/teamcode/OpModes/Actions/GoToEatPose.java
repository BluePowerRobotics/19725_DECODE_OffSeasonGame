package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 前往吃球位姿Action：使用RoadRunner轨迹前往指定的EatPose
 */
public class GoToEatPose implements Action {
    private final Action trajectoryAction;

    public GoToEatPose(MecanumDrive drive, Pose2d eatPose) {
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(eatPose.position, eatPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToEatPose", "Navigating...");
        return trajectoryAction.run(packet);
    }
}