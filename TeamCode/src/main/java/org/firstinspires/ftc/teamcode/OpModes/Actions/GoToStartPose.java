package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 返回起始位姿Action：使用RoadRunner轨迹回到startPose
 */
public class GoToStartPose implements Action {
    private final Action trajectoryAction;

    public GoToStartPose(MecanumDrive drive, Pose2d startPose) {
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(startPose.position, startPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        packet.put("GoToStartPose", "Returning to start...");
        return trajectoryAction.run(packet);
    }
}