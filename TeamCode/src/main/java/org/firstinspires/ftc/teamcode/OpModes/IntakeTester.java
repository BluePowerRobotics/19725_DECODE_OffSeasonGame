package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;

@TeleOp(name = "Intake Tester", group = "Tests")
public class IntakeTester extends LinearOpMode {

    private Sweeper sweeper;
    private Servo triggerServo;
    private boolean triggerAtLaunch = false;

    // Sweeper speed
    private int sweeperSpeed = 0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = -3000;
    private static final int SPEED_MAX = 3000;

    // FullSensor
    private NormalizedColorSensor fullSensor;
    private final float[] fullRawHsv = new float[3];
    private final float[] fullFilteredHsv = new float[3];
    private final float[] fullHsvMin = {360f, 1f, 1f};
    private final float[] fullHsvMax = {0f, 0f, 0f};
    private boolean fullHasData = false;
    private EMA fullEmaH;
    private EMA fullEmaS;
    private EMA fullEmaV;

    // EmptySensor
    private NormalizedColorSensor emptySensor;
    private final float[] emptyRawHsv = new float[3];
    private final float[] emptyFilteredHsv = new float[3];
    private final float[] emptyHsvMin = {360f, 1f, 1f};
    private final float[] emptyHsvMax = {0f, 0f, 0f};
    private boolean emptyHasData = false;
    private EMA emptyEmaH;
    private EMA emptyEmaS;
    private EMA emptyEmaV;

    // Common alpha
    private double currentAlpha = 0.8;
    private static final double ALPHA_STEP = 0.05;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize sweeper
        sweeper = new Sweeper(hardwareMap, telemetry);
        sweeper.setStop();

        // Initialize trigger servo
        triggerServo = hardwareMap.get(Servo.class, "trigger");
        triggerServo.setPosition(HypParams.triggerResetPosition);
        triggerAtLaunch = false;

        // Initialize color sensors
        fullSensor = hardwareMap.get(NormalizedColorSensor.class, "FullSensor");
        emptySensor = hardwareMap.get(NormalizedColorSensor.class, "EmptySensor");

        fullEmaH = new EMA(currentAlpha);
        fullEmaS = new EMA(currentAlpha);
        fullEmaV = new EMA(currentAlpha);
        emptyEmaH = new EMA(currentAlpha);
        emptyEmaS = new EMA(currentAlpha);
        emptyEmaV = new EMA(currentAlpha);

        telemetry.addLine("=== Intake Tester ===");
        telemetry.addLine("D-pad Up/Down: adjust alpha");
        telemetry.addLine("D-pad Left/Right: adjust sweeper speed");
        telemetry.addLine("Left Bumper: trigger launch");
        telemetry.addLine("Right Bumper: trigger reset");
        telemetry.addLine("A: reset FullSensor range & filters");
        telemetry.addLine("B: reset EmptySensor range & filters");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ---- Sweeper speed control (D-pad left/right) ----
            if (gamepad1.dpad_left && !lastDpadLeft) {
                sweeperSpeed = Math.max(SPEED_MIN, sweeperSpeed - SPEED_STEP);
                sweeper.motor.setVelocity(sweeperSpeed);
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                sweeperSpeed = Math.min(SPEED_MAX, sweeperSpeed + SPEED_STEP);
                sweeper.motor.setVelocity(sweeperSpeed);
            }
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

            // ---- Trigger servo control (left/right bumper toggle) ----
            if (gamepad1.left_bumper && !lastLeftBumper) {
                triggerServo.setPosition(HypParams.triggerLaunchPosition);
                triggerAtLaunch = true;
            }
            if (gamepad1.right_bumper && !lastRightBumper) {
                triggerServo.setPosition(HypParams.triggerResetPosition);
                triggerAtLaunch = false;
            }
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            // ---- Alpha control (D-pad up/down) ----
            if (gamepad1.dpad_up && !lastDpadUp) {
                currentAlpha = Math.min(1.0, currentAlpha + ALPHA_STEP);
                fullEmaH.setAlpha(currentAlpha);
                fullEmaS.setAlpha(currentAlpha);
                fullEmaV.setAlpha(currentAlpha);
                emptyEmaH.setAlpha(currentAlpha);
                emptyEmaS.setAlpha(currentAlpha);
                emptyEmaV.setAlpha(currentAlpha);
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                currentAlpha = Math.max(0.0, currentAlpha - ALPHA_STEP);
                fullEmaH.setAlpha(currentAlpha);
                fullEmaS.setAlpha(currentAlpha);
                fullEmaV.setAlpha(currentAlpha);
                emptyEmaH.setAlpha(currentAlpha);
                emptyEmaS.setAlpha(currentAlpha);
                emptyEmaV.setAlpha(currentAlpha);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // ---- Read FullSensor ----
            readSensor(fullSensor, fullEmaH, fullEmaS, fullEmaV, fullRawHsv, fullFilteredHsv);
            updateRange(fullFilteredHsv, fullHsvMin, fullHsvMax, fullHasData);
            fullHasData = true;

            // ---- Read EmptySensor ----
            readSensor(emptySensor, emptyEmaH, emptyEmaS, emptyEmaV, emptyRawHsv, emptyFilteredHsv);
            updateRange(emptyFilteredHsv, emptyHsvMin, emptyHsvMax, emptyHasData);
            emptyHasData = true;

            // ---- Reset ranges ----
            if (gamepad1.a) {
                fullHasData = false;
                fullHsvMin[0] = 360f; fullHsvMin[1] = 1f; fullHsvMin[2] = 1f;
                fullHsvMax[0] = 0f;   fullHsvMax[1] = 0f; fullHsvMax[2] = 0f;
                fullEmaH.reset();
                fullEmaS.reset();
                fullEmaV.reset();
            }
            if (gamepad1.b) {
                emptyHasData = false;
                emptyHsvMin[0] = 360f; emptyHsvMin[1] = 1f; emptyHsvMin[2] = 1f;
                emptyHsvMax[0] = 0f;   emptyHsvMax[1] = 0f; emptyHsvMax[2] = 0f;
                emptyEmaH.reset();
                emptyEmaS.reset();
                emptyEmaV.reset();
            }

            // ---- Telemetry ----
            telemetry.addLine("====== Sweeper & Trigger ======");
            telemetry.addData("Sweeper Speed", "%d RPM", sweeperSpeed);
            telemetry.addData("Trigger Position", triggerAtLaunch ? "LAUNCH (%.3f)" : "RESET (%.3f)",
                    triggerAtLaunch ? HypParams.triggerLaunchPosition : HypParams.triggerResetPosition);

            telemetry.addLine();
            telemetry.addLine("====== FullSensor ======");
            telemetry.addLine()
                    .addData("Raw H", "%.1f", fullRawHsv[0])
                    .addData("S", "%.3f", fullRawHsv[1])
                    .addData("V", "%.3f", fullRawHsv[2]);
            telemetry.addLine()
                    .addData("EMA H", "%.1f", fullFilteredHsv[0])
                    .addData("S", "%.3f", fullFilteredHsv[1])
                    .addData("V", "%.3f", fullFilteredHsv[2]);
            telemetry.addLine("--- FullSensor Range ---");
            telemetry.addLine()
                    .addData("H min", "%.1f", fullHsvMin[0])
                    .addData("H max", "%.1f", fullHsvMax[0]);
            telemetry.addLine()
                    .addData("S min", "%.3f", fullHsvMin[1])
                    .addData("S max", "%.3f", fullHsvMax[1]);
            telemetry.addLine()
                    .addData("V min", "%.3f", fullHsvMin[2])
                    .addData("V max", "%.3f", fullHsvMax[2]);
            telemetry.addData("Full -> isFull", HypParams.isGreenBall(fullFilteredHsv) || HypParams.isPurpleBall(fullFilteredHsv));

            telemetry.addLine();
            telemetry.addLine("====== EmptySensor ======");
            telemetry.addLine()
                    .addData("Raw H", "%.1f", emptyRawHsv[0])
                    .addData("S", "%.3f", emptyRawHsv[1])
                    .addData("V", "%.3f", emptyRawHsv[2]);
            telemetry.addLine()
                    .addData("EMA H", "%.1f", emptyFilteredHsv[0])
                    .addData("S", "%.3f", emptyFilteredHsv[1])
                    .addData("V", "%.3f", emptyFilteredHsv[2]);
            telemetry.addLine("--- EmptySensor Range ---");
            telemetry.addLine()
                    .addData("H min", "%.1f", emptyHsvMin[0])
                    .addData("H max", "%.1f", emptyHsvMax[0]);
            telemetry.addLine()
                    .addData("S min", "%.3f", emptyHsvMin[1])
                    .addData("S max", "%.3f", emptyHsvMax[1]);
            telemetry.addLine()
                    .addData("V min", "%.3f", emptyHsvMin[2])
                    .addData("V max", "%.3f", emptyHsvMax[2]);
            telemetry.addData("Empty -> isEmpty", !(HypParams.isGreenBall(emptyFilteredHsv) || HypParams.isPurpleBall(emptyFilteredHsv)));

            telemetry.addLine();
            telemetry.addData("Alpha", "%.3f", currentAlpha);
            telemetry.addLine("A: reset FullSensor | B: reset EmptySensor");
            telemetry.update();
        }

        // Stop
        sweeper.setStop();
        sweeper.update();
        triggerServo.setPosition(HypParams.triggerResetPosition);
    }

    private void readSensor(NormalizedColorSensor sensor, EMA emaH, EMA emaS, EMA emaV,
                            float[] outRaw, float[] outFiltered) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), outRaw);

        if (!Float.isNaN(outRaw[0]) && !Float.isNaN(outRaw[1]) && !Float.isNaN(outRaw[2])) {
            outFiltered[0] = (float) emaH.update(outRaw[0]);
            outFiltered[1] = (float) emaS.update(outRaw[1]);
            outFiltered[2] = (float) emaV.update(outRaw[2]);
        }
    }

    private void updateRange(float[] filtered, float[] min, float[] max, boolean hasData) {
        if (!hasData) {
            min[0] = max[0] = filtered[0];
            min[1] = max[1] = filtered[1];
            min[2] = max[2] = filtered[2];
        } else {
            if (filtered[0] < min[0]) min[0] = filtered[0];
            if (filtered[0] > max[0]) max[0] = filtered[0];
            if (filtered[1] < min[1]) min[1] = filtered[1];
            if (filtered[1] > max[1]) max[1] = filtered[1];
            if (filtered[2] < min[2]) min[2] = filtered[2];
            if (filtered[2] > max[2]) max[2] = filtered[2];
        }
    }
}