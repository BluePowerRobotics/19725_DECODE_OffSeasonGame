package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utility.HypParams;

public class Tracker {
    private final Detector detector;
    private boolean hasTarget = false;
    private double targetTheta = 0.0;
    private double filteredTheta = 0.0;
    private static final double FilterAlpha = HypParams.FilterAlpha;
    private int stableFrames = 0;

    public Tracker(HardwareMap hardwareMap) {
        this.detector = new Detector(hardwareMap);
    }

    public void start() {
        detector.start();
    }

    public void stop() {
        detector.stop();
    }

    public void update() {
        boolean currentHasTarget = detector.hasTarget();
        double currentBearing = Math.toRadians(detector.bearing());

        if (currentHasTarget) {
            if (!hasTarget) {
                hasTarget = true;
                targetTheta = currentBearing;
                filteredTheta = currentBearing;
                stableFrames = 0;
            } else {
                double delta = currentBearing - filteredTheta;
                if (Math.abs(delta) > HypParams.BearingThreshold) {
                    stableFrames++;
                    if (stableFrames >= HypParams.confirmationFrames) {
                        targetTheta = filteredTheta;
                        stableFrames = 0;
                    }
                } else {
                    stableFrames = 0;
                    filteredTheta = FilterAlpha * currentBearing + (1 - FilterAlpha) * filteredTheta;
                }
            }
        } else {
            hasTarget = false;
        }
    }

    public boolean getHasTarget() {
        return hasTarget;
    }

    public double getTargetTheta() {
        return targetTheta;
    }
}