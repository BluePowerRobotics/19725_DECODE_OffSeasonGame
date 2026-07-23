package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.HypParams;

/**
 * Limelight自主吃球Action：在密道区使用Limelight检测球，引导底盘靠近并吃球
 */
public class LimelightEatAction implements Action {
    private final MecanumDrive drive;
    private final Sweeper sweeper;
    private final Tracker tracker;
    private final long durationMs;
    private long startTimeMs = -1;
    private boolean hasStarted = false;

    private static final double APPROACH_SPEED = 0.8;
    private static final double ROTATION_K = 2.0;
    private static final double MAX_ROTATION = 1.0;

    public LimelightEatAction(MecanumDrive drive, Sweeper sweeper, Tracker tracker, long durationMs) {
        this.drive = drive;
        this.sweeper = sweeper;
        this.tracker = tracker;
        this.durationMs = durationMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (startTimeMs < 0) {
            startTimeMs = System.currentTimeMillis();
            tracker.start();
            hasStarted = true;
        }

        long elapsed = System.currentTimeMillis() - startTimeMs;
        if (elapsed >= durationMs) {
            stop();
            packet.put("LimelightEatAction", "Done (timeout)");
            return false;
        }

        tracker.update();

        double findtime = 0;

        if (tracker.getHasTarget()) {
            findtime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - findtime == 8000) {
            stop();
            packet.put("LimelightEatAction", "Done (ball full)");
            return false;
        }

        sweeper.setEat();
        sweeper.update();

        if (tracker.getHasTarget()) {
            double targetTheta = tracker.getTargetTheta();
            double omega = Math.max(-MAX_ROTATION, Math.min(MAX_ROTATION, targetTheta * ROTATION_K));

            double vx = APPROACH_SPEED * Math.cos(targetTheta);
            double vy = APPROACH_SPEED * Math.sin(targetTheta);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vy, -vx), omega));
            packet.put("LimelightEatAction", "Tracking ball, theta: " + Math.toDegrees(targetTheta));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            packet.put("LimelightEatAction", "No target found");
        }

        return true;
    }

    private void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sweeper.setStop();
        sweeper.update();
        tracker.stop();
    }
}