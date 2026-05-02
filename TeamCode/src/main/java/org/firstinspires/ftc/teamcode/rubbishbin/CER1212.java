package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Compact encoder-based chassis controller.
 * - Assumes a mecanum or 4-wheel drive with encoders on each wheel.
 * - Integrates pose in a field-fixed frame where:
 *     X = forward (meters), Y = right (meters), heading in radians CCW from initial heading.
 * - Keeps implementation minimal and well-encapsulated for testing.
 */
public class CER1212 {

    public static class Pose {
        public double x;
        public double y;
        public double heading; // radians
        public Pose(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
        @Override public String toString() { return String.format("Pose{x=%.3f,y=%.3f,heading=%.3f}", x,y,heading); }
    }

    public static class Constants {
        public final double WHEEL_RADIUS_M; // meters
        public final double TICKS_PER_REV;
        public final double GEAR_RATIO;
        public final double TRACK_WIDTH_M; // distance between left and right wheels
        public final double LATERAL_OFFSET_M; // front-back offset for lateral conversion (if needed)

        public Constants(double wheelRadiusM, double ticksPerRev, double gearRatio, double trackWidthM, double lateralOffsetM) {
            this.WHEEL_RADIUS_M = wheelRadiusM;
            this.TICKS_PER_REV = ticksPerRev;
            this.GEAR_RATIO = gearRatio;
            this.TRACK_WIDTH_M = trackWidthM;
            this.LATERAL_OFFSET_M = lateralOffsetM;
        }

        public static Constants DEFAULT() {
            // Reasonable defaults for many FTC setups; adjust for your robot.
            return new Constants(0.0508, 537.7, 1.0, 0.40, 0.0); // 2in wheel, NeveRest Orbital 20-ish ticks
        }
    }

    private DcMotor lf, rf, lr, rr;
    private final Constants c;

    // internal pose and encoder state
    private Pose pose = new Pose(0,0,0);
    private int lastLf = 0, lastRf = 0, lastLr = 0, lastRr = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Axis alignment: rotate the field axes by this constant when mapping robot-centric deltas to the
    // desired fixed frame. axisAlignmentRadians = 0 means X axis = robot initial forward direction.
    private double axisAlignmentRadians = 0.0;

    public CER1212(HardwareMap hw, Constants constants) {
        this.c = constants != null ? constants : Constants.DEFAULT();
        lf = hw.get(DcMotor.class, "left_front");
        rf = hw.get(DcMotor.class, "right_front");
        lr = hw.get(DcMotor.class, "left_rear");
        rr = hw.get(DcMotor.class, "right_rear");

        // conventional directions - user may need to invert depending on wiring
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // run using encoders for velocity primitives; default to brake
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    /**
     * Fix the global axes so that the positive X axis points at the specified global angle (radians).
     * The angle is measured in the same units as pose.heading (radians CCW from robot initial heading).
     * After calling this, getPose() will report positions and heading in the fixed axes frame.
     */
    public synchronized void setAxisAlignment(double xAxisGlobalAngleRadians) {
        this.axisAlignmentRadians = xAxisGlobalAngleRadians;
    }

    public synchronized double getAxisAlignment() { return this.axisAlignmentRadians; }

    // Utilities
    private double ticksToMeters(int ticks) {
        double revs = ticks / c.TICKS_PER_REV / c.GEAR_RATIO;
        return revs * 2.0 * Math.PI * c.WHEEL_RADIUS_M;
    }

    public synchronized void resetEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // switch back to run using encoders so we can read counts
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastLf = lf.getCurrentPosition();
        lastRf = rf.getCurrentPosition();
        lastLr = lr.getCurrentPosition();
        lastRr = rr.getCurrentPosition();
        pose = new Pose(0,0,0);
        timer.reset();
    }

    /**
     * Return pose in the fixed axes frame (x,y in meters, heading in radians CCW from the fixed X axis).
     */
    public synchronized Pose getPose() { return new Pose(pose.x, pose.y, pose.heading + axisAlignmentRadians); }

    /**
     * Set the pose expressed in the fixed axes frame. heading is CCW from the fixed X axis.
     */
    public synchronized void setPose(double x, double y, double heading) {
        pose.x = x;
        pose.y = y;
        // store internal robot-heading (heading relative to robot-initial-forward)
        pose.heading = heading - axisAlignmentRadians;
    }

    /**
     * Read encoders and integrate pose. Should be called regularly from loop.
     */
    public synchronized void updateFromEncoders() {
        int curLf = lf.getCurrentPosition();
        int curRf = rf.getCurrentPosition();
        int curLr = lr.getCurrentPosition();
        int curRr = rr.getCurrentPosition();

        int dLf = curLf - lastLf;
        int dRf = curRf - lastRf;
        int dLr = curLr - lastLr;
        int dRr = curRr - lastRr;

        lastLf = curLf; lastRf = curRf; lastLr = curLr; lastRr = curRr;

        // Convert wheel deltas to robot-centric dx, dy, dtheta
        double dl = (dLf + dLr) / 2.0; // left wheels average ticks
        double dr = (dRf + dRr) / 2.0; // right wheels average ticks
        double dxTicks = (dl + dr) / 2.0; // forward/back average
        double dyTicks = (-dLf + dRf + dLr - dRr) / 4.0; // lateral component for mecanum-ish behavior approximation
        double dThetaTicks = (dr - dl) / (c.TRACK_WIDTH_M / (2.0 * Math.PI * c.WHEEL_RADIUS_M) * c.TICKS_PER_REV); // approximate

        double dx = ticksToMeters((int)dxTicks);
        double dy = ticksToMeters((int)dyTicks);
        double dTheta = dThetaTicks; // already in radians approx

        // rotate robot-centric dx,dy into the configured fixed frame using (robot heading + axisAlignment)
        double rot = pose.heading + axisAlignmentRadians;
        double sin = Math.sin(rot);
        double cos = Math.cos(rot);
        double fieldDx = cos * dx - sin * dy;
        double fieldDy = sin * dx + cos * dy;

        pose.x += fieldDx;
        pose.y += fieldDy;
        pose.heading += dTheta;
    }

    // Direct motor power control - small convenience
    public void setMotorPowers(double lfP, double rfP, double lrP, double rrP) {
        lf.setPower(lfP);
        rf.setPower(rfP);
        lr.setPower(lrP);
        rr.setPower(rrP);
    }

    public void stop() { setMotorPowers(0,0,0,0); }

    // Simple blocking move: drive in robot-centric forward (x) and right (y) directions a fixed distance in meters
    // This is intentionally minimal: it sets naive open-loop motor powers and waits until target reached per encoders.
    public void moveRobotRelativeMeters(double forwardMeters, double rightMeters, double power) {
        // compute ticks
        int targetLF = lf.getCurrentPosition() + (int)Math.round((forwardMeters - rightMeters) / (2.0 * Math.PI * c.WHEEL_RADIUS_M) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int targetRF = rf.getCurrentPosition() + (int)Math.round((forwardMeters + rightMeters) / (2.0 * Math.PI * c.WHEEL_RADIUS_M) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int targetLR = lr.getCurrentPosition() + (int)Math.round((forwardMeters + rightMeters) / (2.0 * Math.PI * c.WHEEL_RADIUS_M) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int targetRR = rr.getCurrentPosition() + (int)Math.round((forwardMeters - rightMeters) / (2.0 * Math.PI * c.WHEEL_RADIUS_M) * c.TICKS_PER_REV * c.GEAR_RATIO);

        lf.setTargetPosition(targetLF);
        rf.setTargetPosition(targetRF);
        lr.setTargetPosition(targetLR);
        rr.setTargetPosition(targetRR);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPowers(power, power, power, power);

        // wait until motion complete (simple blocking loop) with timeout
        double start = timer.seconds();
        double timeout = Math.max(2.0, Math.abs(forwardMeters) / 0.1 + Math.abs(rightMeters) / 0.1 + 2.0);
        while ( (lf.isBusy() || rf.isBusy() || lr.isBusy() || rr.isBusy()) && (timer.seconds() - start) < timeout) {
            // spin - callers should call updateFromEncoders() externally if they want pose updated during motion
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        stop();

        // back to RUN_USING_ENCODER
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Convenience: return raw encoder counts
    public int[] getEncoderCounts() {
        return new int[]{ lf.getCurrentPosition(), rf.getCurrentPosition(), lr.getCurrentPosition(), rr.getCurrentPosition() };
    }
}
