package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Detector;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 搜索Action：前往SearchPose，等待Limelight检测到3个及以上球
 */
public class SearchAction implements Action {
    private final MecanumDrive drive;
    private final Detector detector;
    private final Action trajectoryAction;
    private boolean trajectoryDone = false;

    public SearchAction(MecanumDrive drive, Pose2d searchPose, Detector detector) {
        this.drive = drive;
        this.detector = detector;
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        this.trajectoryAction = drive.actionBuilder(currentPose)
                .splineToConstantHeading(searchPose.position, searchPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!trajectoryDone) {
            trajectoryDone = !trajectoryAction.run(packet);
            if (!trajectoryDone) {
                packet.put("SearchAction", "Arrived, waiting for balls...");
            }
            return true;
        }
        // 等待检测到3个及以上的球
        int count = detector.targetNum();
        packet.put("SearchAction", "Waiting, detected: " + count);
        return count < 3;
    }
}