package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Compact encoder-based chassis controller with matrix-based mecanum kinematics and simple IMU fusion.
 * - Robot frame: X = forward, Y = right, heading CCW positive.
 * - getPose()/setPose() use the fixed axes (affected by axisAlignmentRadians).
 */
public class ChassisController23 {

    public static class Pose {
        public double x, y, heading; // meters, meters, radians (fixed-axis)
        public Pose(double x, double y, double heading) { this.x = x; this.y = y; this.heading = heading; }
        @NonNull
        public String toString() { return String.format(Locale.CHINA, "Pose{x=%.3f,y=%.3f,heading=%.3f}", x, y, heading); }
    }

    public static class Constants {
        public final double WHEEL_RADIUS_M; // meters
        public final double TICKS_PER_REV;
        public final double GEAR_RATIO;
        public final double TRACK_WIDTH_M; // left-right distance
        public final double LATERAL_OFFSET_M; // front-back half-length (for rotation lever arm)

        public Constants(double wheelRadiusM, double ticksPerRev, double gearRatio, double trackWidthM, double lateralOffsetM) {
            this.WHEEL_RADIUS_M = wheelRadiusM;
            this.TICKS_PER_REV = ticksPerRev;
            this.GEAR_RATIO = gearRatio;
            this.TRACK_WIDTH_M = trackWidthM;
            this.LATERAL_OFFSET_M = lateralOffsetM;
        }

        public static Constants DEFAULT() { return new Constants(0.0508, 537.7, 1.0, 0.40, 0.0); }
    }

    private final DcMotor lf, rf, lr, rr;
    private final Constants c;

    // pose stored in fixed axes frame internally
    private Pose pose = new Pose(0, 0, 0);
    private int lastLf, lastRf, lastLr, lastRr;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastUpdateTime = 0.0;

    // axis alignment: rotate fixed axes by this angle relative to robot initial forward
    private double axisAlignmentRadians = 0.0;

    // IMU
    private BNO055IMU imu = null;
    private boolean imuPresent;

    // non-blocking move state
    private volatile boolean moveBusy = false;
    private double moveTargetX, moveTargetY, moveTargetHeading;
    private double movePosTol = 0.05; // meters
    private double moveHeadingTol = Math.toRadians(5.0);
    private double moveMaxPower = 0.6;
    private final double moveMinPower = 0.12;

    // lightweight PID gains storage so tester can tune P/I/D remotely
    // order: P, I, D
    private final float[] gainsVx = new float[]{1.0f, 0.0f, 0.0f};
    private final float[] gainsVy = new float[]{1.0f, 0.0f, 0.0f};
    private final float[] gainsW  = new float[]{1.0f, 0.0f, 0.0f};

    // simple PID state for each axis
    private double integVx = 0.0, prevVx = 0.0;
    private double integVy = 0.0, prevVy = 0.0;
    private double integW  = 0.0, prevW  = 0.0;

    public ChassisController23(HardwareMap hw, Constants constants) {
        this.c = constants != null ? constants : Constants.DEFAULT();

        lf = hw.get(DcMotor.class, "left_front");
        rf = hw.get(DcMotor.class, "right_front");
        lr = hw.get(DcMotor.class, "left_rear");
        rr = hw.get(DcMotor.class, "right_rear");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // try to init IMU if present under the common name "imu"; tolerate absence
        try {
            imu = hw.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters p = new BNO055IMU.Parameters();
            p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            p.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imu.initialize(p);
            imuPresent = true;
        } catch (Exception ignored) { imuPresent = false; }

        // default imuWeight already set in declaration
        resetEncoders();
    }

    public synchronized void setAxisAlignment(double xAxisGlobalAngleRadians) { this.axisAlignmentRadians = xAxisGlobalAngleRadians; }
    public synchronized double getAxisAlignment() { return axisAlignmentRadians; }

    // PID gain accessors for tester tuning and persistence
    public synchronized void setPidVxGains(double p, double i, double d) { gainsVx[0] = (float)p; gainsVx[1] = (float)i; gainsVx[2] = (float)d; }
    public synchronized void setPidVyGains(double p, double i, double d) { gainsVy[0] = (float)p; gainsVy[1] = (float)i; gainsVy[2] = (float)d; }
    public synchronized void setPidWGains(double p, double i, double d)  { gainsW[0] = (float)p; gainsW[1] = (float)i; gainsW[2] = (float)d; }
    public synchronized float[] getPidVxGains() { return gainsVx.clone(); }
    public synchronized float[] getPidVyGains() { return gainsVy.clone(); }
    public synchronized float[] getPidWGains()  { return gainsW.clone(); }

    private double ticksToMeters(double ticks) {
        double revs = ticks / c.TICKS_PER_REV / c.GEAR_RATIO;
        return revs * 2.0 * Math.PI * c.WHEEL_RADIUS_M;
    }

    public synchronized void resetEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastLf = lf.getCurrentPosition();
        lastRf = rf.getCurrentPosition();
        lastLr = lr.getCurrentPosition();
        lastRr = rr.getCurrentPosition();

        pose = new Pose(0, 0, imuPresent ? getImuInternal() : 0.0);
        timer.reset();
        lastUpdateTime = timer.seconds();
        integVx = integVy = integW = 0.0; prevVx = prevVy = prevW = 0.0;
    }

    public synchronized Pose getPose() { return new Pose(pose.x, pose.y, pose.heading); }
    public synchronized void setPose(double x, double y, double heading) { pose.x = x; pose.y = y; pose.heading = heading; }

    // IMU heading in the same internal frame as pose.heading (fixed-axis internal)
    private double getImuInternal() {
        if (!imuPresent) return 0.0;
        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return o.firstAngle - axisAlignmentRadians; // internal fixed-axis heading
    }

    /**
     * Update pose using matrix-based mecanum kinematics and complementary filter for heading.
     * IMU is used as a slow-correcting measurement (imuWeight).
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

        double sLf = ticksToMeters(dLf);
        double sRf = ticksToMeters(dRf);
        double sLr = ticksToMeters(dLr);
        double sRr = ticksToMeters(dRr);

        double vx = (sLf + sRf + sLr + sRr) / 4.0;
        double vy = (-sLf + sRf + sLr - sRr) / 4.0;
        double trackHalf = c.TRACK_WIDTH_M / 2.0;
        double lever = trackHalf + c.LATERAL_OFFSET_M;
        double omega = 0.0;
        if (lever != 0.0) omega = (-sLf + sRf - sLr + sRr) / (4.0 * lever);

        // predicted heading from encoders (internal frame)
        double predictedHeading = pose.heading + omega;

        if (imuPresent) {
            double imuInternal = getImuInternal();
            // complementary fusion: short-term stick with encoder prediction, long-term correct with IMU
            // complementary filter weight for IMU (small => short-term encoder favored)
            double imuWeight = 0.02;
            pose.heading = imuWeight * imuInternal + (1.0 - imuWeight) * predictedHeading;
        } else {
            pose.heading = predictedHeading;
        }

        // rotate robot-centric dx,dy into fixed frame using fused heading
        double rot = pose.heading + axisAlignmentRadians; // absolute heading for rotation
        double sin = Math.sin(rot);
        double cos = Math.cos(rot);

        double fieldDx = cos * vx - sin * vy;
        double fieldDy = sin * vx + cos * vy;

        pose.x += fieldDx;
        pose.y += fieldDy;
    }

    // Motor low-level control
    public void setMotorPowers(double lfP, double rfP, double lrP, double rrP) { lf.setPower(lfP); rf.setPower(rfP); lr.setPower(lrP); rr.setPower(rrP); }
    public void stop() { setMotorPowers(0, 0, 0, 0); }

    // Non-blocking relative move (calls absolute move under the hood)
    public synchronized void moveToStart(double forwardMeters, double rightMeters, double power) {
        // convert robot-relative to field-relative target
        double rot = pose.heading + axisAlignmentRadians; // absolute
        double fx = Math.cos(rot) * forwardMeters - Math.sin(rot) * rightMeters;
        double fy = Math.sin(rot) * forwardMeters + Math.cos(rot) * rightMeters;
        double tx = pose.x + fx;
        double ty = pose.y + fy;
        moveToAbsolute(tx, ty, pose.heading, power, movePosTol, moveHeadingTol);
    }

    // New: non-blocking absolute move with tolerances and simple power ramping
    public synchronized void moveToAbsolute(double targetX, double targetY, double targetHeading, double maxPower, double posTol, double headingTol) {
        if (moveBusy) return;
        this.moveTargetX = targetX; this.moveTargetY = targetY; this.moveTargetHeading = targetHeading;
        this.movePosTol = posTol; this.moveHeadingTol = headingTol; this.moveMaxPower = maxPower;

        // compute relative robot-frame distances at start
        double dx = targetX - pose.x; double dy = targetY - pose.y;
        double rot = pose.heading + axisAlignmentRadians; // absolute
        // field -> robot
        double forward = Math.cos(rot) * dx + Math.sin(rot) * dy;
        double right = -Math.sin(rot) * dx + Math.cos(rot) * dy;

        int curLf = lf.getCurrentPosition(); int curRf = rf.getCurrentPosition(); int curLr = lr.getCurrentPosition(); int curRr = rr.getCurrentPosition();
        int deltaLF = (int)Math.round(((forward - right) / (2.0 * Math.PI * c.WHEEL_RADIUS_M)) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int deltaRF = (int)Math.round(((forward + right) / (2.0 * Math.PI * c.WHEEL_RADIUS_M)) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int deltaLR = (int)Math.round(((forward + right) / (2.0 * Math.PI * c.WHEEL_RADIUS_M)) * c.TICKS_PER_REV * c.GEAR_RATIO);
        int deltaRR = (int)Math.round(((forward - right) / (2.0 * Math.PI * c.WHEEL_RADIUS_M)) * c.TICKS_PER_REV * c.GEAR_RATIO);

        lf.setTargetPosition(curLf + deltaLF);
        rf.setTargetPosition(curRf + deltaRF);
        lr.setTargetPosition(curLr + deltaLR);
        rr.setTargetPosition(curRr + deltaRR);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start with maxPower
        double p = Math.max(moveMinPower, Math.min(moveMaxPower, maxPower));
        setMotorPowers(p, p, p, p);
        moveBusy = true;
    }

    public synchronized boolean isMoveBusy() {
        if (!moveBusy) return false;
        // compute remaining distance to target (field frame)
        double dx = moveTargetX - pose.x; double dy = moveTargetY - pose.y;
        double rem = Math.hypot(dx, dy);
        double dHeading = normalizeHeading(moveTargetHeading - pose.heading);

        // ramp power based on remaining distance
        double p = moveMaxPower;
        // meters to start slowing
        double moveSlowDist = 0.25;
        if (rem < moveSlowDist) {
            p = moveMinPower + (moveMaxPower - moveMinPower) * (rem / moveSlowDist);
        }
        p = Math.max(moveMinPower, Math.min(moveMaxPower, p));
        // compute robot-frame remaining forward/right for proportional control
        double rot = pose.heading + axisAlignmentRadians;
        double forward = Math.cos(rot) * dx + Math.sin(rot) * dy;
        double right = -Math.sin(rot) * dx + Math.cos(rot) * dy;
        // PID compute using stored gains (P/I/D). Integrators are simple and reset on resetEncoders().
        double now = timer.seconds();
        double dt = now - lastUpdateTime; if (dt <= 0) dt = 1e-6;

        // Vx axis
        double errVx = forward;
        integVx += errVx * dt;
        double derivVx = (errVx - prevVx) / dt; prevVx = errVx;
        double vxCmdUnclamped = gainsVx[0] * errVx + gainsVx[1] * integVx + gainsVx[2] * derivVx;
        double vxCmd = clampUnit(vxCmdUnclamped) * p;

        // Vy axis
        double errVy = right;
        integVy += errVy * dt;
        double derivVy = (errVy - prevVy) / dt; prevVy = errVy;
        double vyCmdUnclamped = gainsVy[0] * errVy + gainsVy[1] * integVy + gainsVy[2] * derivVy;
        double vyCmd = clampUnit(vyCmdUnclamped) * p;

        // W axis
        double errW = dHeading;
        integW += errW * dt;
        double derivW = (errW - prevW) / dt; prevW = errW;
        double wCmdUnclamped = gainsW[0] * errW + gainsW[1] * integW + gainsW[2] * derivW;
        double wCmd = clampUnit(wCmdUnclamped) * p;

        // wheel mixing
        double lfP = vxCmd + vyCmd + wCmd;
        double rfP = vxCmd - vyCmd - wCmd;
        double lrP = vxCmd - vyCmd + wCmd;
        double rrP = vxCmd + vyCmd - wCmd;
        double max = Math.max(1.0, Math.max(Math.abs(lfP), Math.max(Math.abs(rfP), Math.max(Math.abs(lrP), Math.abs(rrP)))));
        lfP /= max; rfP /= max; lrP /= max; rrP /= max;
        setMotorPowers(lfP, rfP, lrP, rrP);

        // check finish conditions
        boolean withinPos = rem <= movePosTol;
        boolean withinHead = Math.abs(dHeading) <= moveHeadingTol;

        if (withinPos && withinHead) {
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stop();
            moveBusy = false;
        }
        lastUpdateTime = now;
        return moveBusy;
    }

    private double clampUnit(double v) { return Math.max(-1.0, Math.min(1.0, v)); }

    private double normalizeHeading(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // Explicit update to be called in loop; updates pose and checks move state
    public void updateMove() {
        updateFromEncoders();
        isMoveBusy();
    }

    public int[] getEncoderCounts() { return new int[]{ lf.getCurrentPosition(), rf.getCurrentPosition(), lr.getCurrentPosition(), rr.getCurrentPosition() }; }
}
