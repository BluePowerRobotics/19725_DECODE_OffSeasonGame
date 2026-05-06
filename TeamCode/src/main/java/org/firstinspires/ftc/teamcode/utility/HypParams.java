package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.roadrunner.Pose2d;



public class HypParams {
    public static ConvexPolygon BoundingBox = new ConvexPolygon(
        new Point2D(20, 20),
        new Point2D(-20, 20),
        new Point2D(-20, -20),
        new Point2D(20, -20)
    );

    public static ConvexPolygon SHOOTING_AREA_LEFT = new ConvexPolygon(
        new Point2D(-72, 12),
        new Point2D(-72, -12),
        new Point2D(-60, 0)
    );

    public static ConvexPolygon SHOOTING_AREA_RIGHT = new ConvexPolygon(
        new Point2D(0, 0),
        new Point2D(72, 72),
        new Point2D(72, -72)
    );

    public static double deltaH = 0.5;

    public static double shooterK = 1.0;

    public static double shooterB = 0.0;

    public static double maxV = 0.5;

    public static double maxOmega = Math.PI / 2;

    public static Pose2d startPoseRed = new Pose2d(0, 0, 0);

    public static Pose2d startPoseBlue = new Pose2d(0, 0, 0);

    public static double WanderSpeed = 1.0;

    public static double Limelight_h = 0.0;

    public static double Limelight_m0 = 0.0;

    public static boolean InitialRunningToPose = true;

    public static double distanceThreshold = 0.15;

    public static int confirmationFrames = 3;

    public static int removalFrames = 5;
}