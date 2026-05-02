package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp(name = "ChassisTester23", group = "Test")
@SuppressWarnings("unused")
public class ChassisTester23 extends LinearOpMode {

    private static boolean isRunning = false;
    private SharedPreferences prefs;
    private static final String PREF_KEY = "chassis_pid";

    // tuning state
    private int tuneAxis = 0; // 0=vx,1=vy,2=w
    private int tuneParam = 0; // 0=P,1=I,2=D

    @Override
    public void runOpMode() {
        ChassisController23.Constants constants = ChassisController23.Constants.DEFAULT();
        ChassisController23 chassis = new ChassisController23(hardwareMap, constants);

        // load persistent gains from SharedPreferences
        Context ctx = hardwareMap.appContext;
        prefs = ctx.getSharedPreferences(PREF_KEY, Context.MODE_PRIVATE);
        loadGains(chassis);

        telemetry.addData("status", "Initialized");
        telemetry.update();

        waitForStart();

        // default reset
        chassis.resetEncoders();
        chassis.setPose(0, 0, 0);

        // PID tuning controls with edge detection
        final double coarse = 0.1, fine = 0.01;
        boolean prevDpadLeft=false, prevDpadRight=false, prevDpadUp=false, prevDpadDown=false;
        boolean prevLeftBumper=false, prevRightBumper=false, prevStart=false;
        boolean prevB=false;

        while (opModeIsActive()) {
            // update localization and move state
            chassis.updateMove();

            // toggle teleop driving (B press edge)
            if (gamepad1.b && !prevB) { // use B but simple edge via prevA to avoid rapid toggles
                isRunning = !isRunning;
            }

            // start simple non-blocking moves with buttons when not busy
            if (gamepad1.x && !chassis.isMoveBusy()) {
                // forward 0.5m
                chassis.moveToStart(0.5, 0.0, 0.6);
            }
            if (gamepad1.y && !chassis.isMoveBusy()) {
                // strafe right 0.5m
                chassis.moveToStart(0.0, 0.5, 0.6);
            }
            if (gamepad1.a) {
                // reset encoders & pose
                chassis.resetEncoders();
                chassis.setPose(0, 0, 0);
            }

            boolean curDpadLeft = gamepad1.dpad_left;
            boolean curDpadRight = gamepad1.dpad_right;
            boolean curDpadUp = gamepad1.dpad_up;
            boolean curDpadDown = gamepad1.dpad_down;
            boolean curLeftBumper = gamepad1.left_bumper;
            boolean curRightBumper = gamepad1.right_bumper;
            boolean curStart = gamepad1.start;

            // axis switch
            if (curDpadLeft && !prevDpadLeft) { tuneAxis = (tuneAxis + 2) % 3; }
            if (curDpadRight && !prevDpadRight) { tuneAxis = (tuneAxis + 1) % 3; }
            // param switch
            if (curDpadUp && !prevDpadUp) { tuneParam = (tuneParam + 1) % 3; }
            if (curDpadDown && !prevDpadDown) { tuneParam = (tuneParam + 2) % 3; }

            // adjust gains: left bumper increase coarse, right bumper decrease coarse; hold start+bumper for fine
            if (curLeftBumper && !prevLeftBumper) {
                adjustGain(chassis, +coarse);
            }
            if (curRightBumper && !prevRightBumper) {
                adjustGain(chassis, -coarse);
            }
            // fine adjustments while holding start
            if (curStart && curLeftBumper) { adjustGain(chassis, +fine); }
            if (curStart && curRightBumper) { adjustGain(chassis, -fine); }

            // save gains when pressing start alone (edge)
            if (curStart && !prevStart) {
                saveGains(chassis);
            }

            prevDpadLeft = curDpadLeft; prevDpadRight = curDpadRight; prevDpadUp = curDpadUp; prevDpadDown = curDpadDown;
            prevLeftBumper = curLeftBumper; prevRightBumper = curRightBumper; prevStart = curStart;
            prevB = gamepad1.b;

            // teleop driving when isRunning and no active RUN_TO_POSITION move
            if (isRunning && !chassis.isMoveBusy()) {
                double vx = -gamepad1.left_stick_y; // forward
                double vy = gamepad1.left_stick_x;  // right
                double omega = gamepad1.right_stick_x; // rotation

                // simple mecanum power mixing
                double lf = vx + vy + omega;
                double rf = vx - vy - omega;
                double lr = vx - vy + omega;
                double rr = vx + vy - omega;
                double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));
                lf /= max; rf /= max; lr /= max; rr /= max;
                chassis.setMotorPowers(lf, rf, lr, rr);
            }

            // telemetry
            ChassisController23.Pose p = chassis.getPose();
            telemetry.addData("pose", String.format(Locale.US, "x=%.2f y=%.2f h=%.2f", p.x, p.y, p.heading));
            telemetry.addData("encoders", java.util.Arrays.toString(chassis.getEncoderCounts()));
            telemetry.addData("moveBusy", chassis.isMoveBusy());
            telemetry.addData("isRunning", isRunning);

            float[] vxg = chassis.getPidVxGains(); float[] vyg = chassis.getPidVyGains(); float[] wg = chassis.getPidWGains();
            telemetry.addData("Gains Vx", String.format(Locale.US, "P=%.3f I=%.3f D=%.3f", vxg[0], vxg[1], vxg[2]));
            telemetry.addData("Gains Vy", String.format(Locale.US, "P=%.3f I=%.3f D=%.3f", vyg[0], vyg[1], vyg[2]));
            telemetry.addData("Gains W",  String.format(Locale.US, "P=%.3f I=%.3f D=%.3f", wg[0], wg[1], wg[2]));

            telemetry.addData("tuneAxis", tuneAxis == 0 ? "Vx" : tuneAxis == 1 ? "Vy" : "W");
            telemetry.addData("tuneParam", tuneParam == 0 ? "P" : tuneParam == 1 ? "I" : "D");
            telemetry.addData("hint", "DPad L/R axis, DPad U/D param, LB/RB coarse, Start+LB/RB fine, Start save");

            telemetry.update();

            idle();
        }
    }

    private void adjustGain(ChassisController23 chassis, double delta) {
        float[] g;
        switch (tuneAxis) {
            case 0:
                g = chassis.getPidVxGains(); g[tuneParam] = (float)(g[tuneParam] + delta); chassis.setPidVxGains(g[0], g[1], g[2]); break;
            case 1:
                g = chassis.getPidVyGains(); g[tuneParam] = (float)(g[tuneParam] + delta); chassis.setPidVyGains(g[0], g[1], g[2]); break;
            default:
                g = chassis.getPidWGains();  g[tuneParam] = (float)(g[tuneParam] + delta); chassis.setPidWGains(g[0], g[1], g[2]); break;
        }
    }

    private void saveGains(ChassisController23 chassis) {
        SharedPreferences.Editor e = prefs.edit();
        float[] vxg = chassis.getPidVxGains(); float[] vyg = chassis.getPidVyGains(); float[] wg = chassis.getPidWGains();
        e.putFloat("vx_p", vxg[0]); e.putFloat("vx_i", vxg[1]); e.putFloat("vx_d", vxg[2]);
        e.putFloat("vy_p", vyg[0]); e.putFloat("vy_i", vyg[1]); e.putFloat("vy_d", vyg[2]);
        e.putFloat("w_p", wg[0]);  e.putFloat("w_i", wg[1]);  e.putFloat("w_d", wg[2]);
        e.apply();
        telemetry.addData("save", "gains saved"); telemetry.update();
    }

    private void loadGains(ChassisController23 chassis) {
        float vxp = prefs.getFloat("vx_p", 1.0f); float vxi = prefs.getFloat("vx_i", 0.0f); float vxd = prefs.getFloat("vx_d", 0.0f);
        float vyp = prefs.getFloat("vy_p", 1.0f); float vyi = prefs.getFloat("vy_i", 0.0f); float vyd = prefs.getFloat("vy_d", 0.0f);
        float wp  = prefs.getFloat("w_p", 1.0f);  float wi  = prefs.getFloat("w_i", 0.0f);  float wd  = prefs.getFloat("w_d", 0.0f);
        chassis.setPidVxGains(vxp, vxi, vxd);
        chassis.setPidVyGains(vyp, vyi, vyd);
        chassis.setPidWGains(wp, wi, wd);
    }
}
