package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;

@TeleOp(name = "TrackerTest", group = "Tests")
public class TrackerTest extends LinearOpMode {
    private Tracker tracker;
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL = 1000;

    @Override
    public void runOpMode() {
        tracker = new Tracker(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press play to start tracking targets");
        telemetry.update();

        waitForStart();

        tracker.start();

        while (opModeIsActive()) {
            tracker.update();

            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
                lastUpdateTime = currentTime;

                telemetry.addData("Status", "Running");
                telemetry.addData("Has Target", tracker.getHasTarget());
                telemetry.addData("Target Theta", "%.4f rad (%.2f deg)",
                        tracker.getTargetTheta(), Math.toDegrees(tracker.getTargetTheta()));
                tracker.printAll();
                telemetry.update();
            }
        }

        tracker.stop();
    }
}