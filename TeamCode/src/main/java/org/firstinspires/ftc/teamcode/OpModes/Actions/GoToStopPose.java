package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;

public class GoToStopPose implements Action {
    private final Chassis chassis;
    private final Sweeper sweeper;
    private final Pose2d targetPose;
    private Action trajectoryAction;
    private boolean trajectoryStarted;

    public GoToStopPose(Chassis chassis, Sweeper sweeper, Pose2d targetPose) {
        this.chassis = chassis;
        this.sweeper = sweeper;
        this.targetPose = targetPose;
        this.trajectoryStarted = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        sweeper.setStop();
        sweeper.update();

        if (!trajectoryStarted) {
            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
            trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                    .splineToLinearHeading(targetPose, currentPose.heading.toDouble())
                    .build();
            trajectoryStarted = true;
        }

        if (trajectoryAction != null) {
            boolean running = trajectoryAction.run(packet);
            if (!running) {
                trajectoryStarted = false;
                trajectoryAction = null;
                return false;
            }
        }

        return true;
    }
}
