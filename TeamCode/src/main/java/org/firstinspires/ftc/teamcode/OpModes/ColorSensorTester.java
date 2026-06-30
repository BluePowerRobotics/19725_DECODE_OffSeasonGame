package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Tester", group = "Sensor")
public class ColorSensorTester extends LinearOpMode {

    // HSV bounds: 0=hue, 1=saturation, 2=value
    private final float[] hsvMin = {360f, 1f, 1f};
    private final float[] hsvMax = {0f, 0f, 0f};
    private boolean hasData = false;

    @Override
    public void runOpMode() {
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        final float[] hsvValues = new float[3];

        telemetry.addData("Status", "Initialized. Place sensor over a ball and press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read normalized RGBA
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Track running min/max
            if (!hasData) {
                hsvMin[0] = hsvMax[0] = hsvValues[0];
                hsvMin[1] = hsvMax[1] = hsvValues[1];
                hsvMin[2] = hsvMax[2] = hsvValues[2];
                hasData = true;
            } else {
                if (hsvValues[0] < hsvMin[0]) hsvMin[0] = hsvValues[0];
                if (hsvValues[0] > hsvMax[0]) hsvMax[0] = hsvValues[0];
                if (hsvValues[1] < hsvMin[1]) hsvMin[1] = hsvValues[1];
                if (hsvValues[1] > hsvMax[1]) hsvMax[1] = hsvValues[1];
                if (hsvValues[2] < hsvMin[2]) hsvMin[2] = hsvValues[2];
                if (hsvValues[2] > hsvMax[2]) hsvMax[2] = hsvValues[2];
            }

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
            telemetry.addLine("--- Observed HSV Range ---");
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
            telemetry.addLine("Press A to reset HSV range");

            if (gamepad1.a) {
                hasData = false;
            }

            telemetry.update();
        }
    }
}