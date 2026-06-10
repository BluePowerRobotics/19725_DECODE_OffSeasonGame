package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.HypParams;

public class Tracker {
    private final Detector detector;
    private boolean hasTarget = false;
    private double targetTheta = 0.0;
    private double filteredTheta = 0.0;
    private static final double FilterAlpha = HypParams.FilterAlpha;
    private int stableFrames = 0;
    private int appearanceFrames = 0;
    private int disappearanceFrames = 0;

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
            appearanceFrames++;
            disappearanceFrames = 0;

            if (appearanceFrames >= HypParams.confirmationFrames) {
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
                            filteredTheta = currentBearing;
                            targetTheta = currentBearing;
                            stableFrames = 0;
                        }
                    } else {
                        stableFrames = 0;
                        filteredTheta = FilterAlpha * currentBearing + (1 - FilterAlpha) * filteredTheta;
                        targetTheta = filteredTheta;
                    }
                }
            }
        } else {
            disappearanceFrames++;
            appearanceFrames = 0;

            if (disappearanceFrames >= HypParams.removalFrames) {
                hasTarget = false;
            }
        }
    }
    public void printAll(Telemetry telemetry){
        java.util.List<String> allObjectsInfo = detector.PrintAll();
        telemetry.addData("All Objects", "Count: " + (allObjectsInfo.size() > 0 && allObjectsInfo.get(0).equals("No valid Limelight results") ? 0 : allObjectsInfo.size()));
        for (String info : allObjectsInfo) {
            telemetry.addData("Object", info);
        }

        telemetry.update();
    }
    public boolean getHasTarget() {
        return hasTarget;
    }

    public double getTargetTheta() {
        return -targetTheta;
    }
}