package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utility.filter.EMA;

@TeleOp(name = "Color Sensor Tester", group = "Sensor")
public class ColorSensorTester extends LinearOpMode {

    // HSV bounds: 0=hue, 1=saturation, 2=value
    private final float[] hsvMin = {360f, 1f, 1f};
    private final float[] hsvMax = {0f, 0f, 0f};
    private boolean hasData = false;

    // EMA filters for H, S, V
    private EMA emaH;
    private EMA emaS;
    private EMA emaV;
    private double currentAlpha = 0.8;
    private final double alphaStep = 0.05;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        final float[] hsvValues = new float[3];
        final double[] filteredHsv = new double[3];

        emaH = new EMA(currentAlpha);
        emaS = new EMA(currentAlpha);
        emaV = new EMA(currentAlpha);

        telemetry.addData("Status", "Initialized. Place sensor over a ball and press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read normalized RGBA
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Update EMA filters
            filteredHsv[0] = emaH.update(hsvValues[0]);
            filteredHsv[1] = emaS.update(hsvValues[1]);
            filteredHsv[2] = emaV.update(hsvValues[2]);

            // Track running min/max of filtered values
            if (!hasData) {
                hsvMin[0] = hsvMax[0] = (float) filteredHsv[0];
                hsvMin[1] = hsvMax[1] = (float) filteredHsv[1];
                hsvMin[2] = hsvMax[2] = (float) filteredHsv[2];
                hasData = true;
            } else {
                if (filteredHsv[0] < hsvMin[0]) hsvMin[0] = (float) filteredHsv[0];
                if (filteredHsv[0] > hsvMax[0]) hsvMax[0] = (float) filteredHsv[0];
                if (filteredHsv[1] < hsvMin[1]) hsvMin[1] = (float) filteredHsv[1];
                if (filteredHsv[1] > hsvMax[1]) hsvMax[1] = (float) filteredHsv[1];
                if (filteredHsv[2] < hsvMin[2]) hsvMin[2] = (float) filteredHsv[2];
                if (filteredHsv[2] > hsvMax[2]) hsvMax[2] = (float) filteredHsv[2];
            }

            // Adjust alpha with D-pad (edge detection)
            if (gamepad1.dpad_up && !lastDpadUp) {
                currentAlpha = Math.min(1.0, currentAlpha + alphaStep);
                emaH.setAlpha(currentAlpha);
                emaS.setAlpha(currentAlpha);
                emaV.setAlpha(currentAlpha);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                currentAlpha = Math.max(0.0, currentAlpha - alphaStep);
                emaH.setAlpha(currentAlpha);
                emaS.setAlpha(currentAlpha);
                emaV.setAlpha(currentAlpha);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // ---- Telemetry ----
            telemetry.addLine("--- Current Reading ---");
            telemetry.addLine()
                    .addData("R", "%.3f", colors.red)
                    .addData("G", "%.3f", colors.green)
                    .addData("B", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("H", "%.1f", hsvValues[0])
                    .addData("S", "%.3f", hsvValues[1])
                    .addData("V", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addLine();
            telemetry.addLine("--- EMA Filtered HSV---");
            telemetry.addLine()
                    .addData("H", "%.1f", filteredHsv[0])
                    .addData("S", "%.3f", filteredHsv[1])
                    .addData("V", "%.3f", filteredHsv[2])
                    .addData("alpha", "%.3f", currentAlpha);

            telemetry.addLine();
            telemetry.addLine("--- Observed Filtered HSV Range ---");
            telemetry.addLine()
                    .addData("H min", "%.1f", hsvMin[0])
                    .addData("H max", "%.1f", hsvMax[0]);
            telemetry.addLine()
                    .addData("S min", "%.3f", hsvMin[1])
                    .addData("S max", "%.3f", hsvMax[1]);
            telemetry.addLine()
                    .addData("V min", "%.3f", hsvMin[2])
                    .addData("V max", "%.3f", hsvMax[2]);

            telemetry.addLine();
            telemetry.addLine("D-pad Up/Down: adjust alpha");
            telemetry.addLine("Press A to reset HSV range & filters");

            if (gamepad1.a) {
                hasData = false;
                emaH.reset();
                emaS.reset();
                emaV.reset();
            }

            telemetry.update();
        }
    }
}