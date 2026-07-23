package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

/**
 * 前往射击区域Action：红色前往(-24, 24, PI)，蓝色前往(-24, -24, PI)
 */
public class GoToShootingAreaAction implements Action {
    private static final Pose2d TARGET_POSE_RED = new Pose2d(-24, 24, Math.PI);
    private static final Pose2d TARGET_POSE_BLUE = new Pose2d(-24, -24, Math.PI);
    private final Action trajectoryAction;

    public GoToShootingAreaAction(MecanumDrive drive, TeamColor teamColor) {
        Pose2d targetPose = teamColor == TeamColor.RED ? TARGET_POSE_RED : TARGET_POSE_BLUE;
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(targetPose.position.x, targetPose.position.y), targetPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToShootingArea", "Navigating...");
        return trajectoryAction.run(packet);
    }
}