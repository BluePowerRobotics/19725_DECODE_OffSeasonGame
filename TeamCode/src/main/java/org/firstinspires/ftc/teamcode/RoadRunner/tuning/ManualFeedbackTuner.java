package org.firstinspires.ftc.teamcode.RoadRunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.TankDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.TwoDeadWheelLocalizer;

@Config
public final class ManualFeedbackTuner extends LinearOpMode {
    public static String MODE = "X";
    public static double DISTANCE = 64;
    public static double DEGREE = 3.1415926;
    public static double TIME = 1;

    private void checkOdometry(Object drive) {
        if (drive instanceof MecanumDrive) {
            if (((MecanumDrive) drive).localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (((MecanumDrive) drive).localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
        } else if (drive instanceof TankDrive) {
            if (((TankDrive) drive).localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (((TankDrive) drive).localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            checkOdometry(drive);
            waitForStart();

            while (opModeIsActive()) {
                switch (MODE) {
                    case "X":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .lineToX(DISTANCE)
                                        .waitSeconds(TIME)
                                        .lineToX(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                    case "Y":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .lineToY(DISTANCE)
                                        .waitSeconds(TIME)
                                        .lineToY(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                    case "H":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .turnTo(DEGREE)
                                        .waitSeconds(TIME)
                                        .turnTo(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            checkOdometry(drive);
            waitForStart();

            while (opModeIsActive()) {
                switch (MODE) {
                    case "X":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .lineToX(DISTANCE)
                                        .waitSeconds(TIME)
                                        .lineToX(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                    case "Y":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .lineToY(DISTANCE)
                                        .waitSeconds(TIME)
                                        .lineToY(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                    case "H":
                        Actions.runBlocking(
                                drive.actionBuilder(new Pose2d(0, 0, 0))
                                        .turnTo(DEGREE)
                                        .waitSeconds(TIME)
                                        .turnTo(0)
                                        .waitSeconds(TIME)
                                        .build());
                        break;
                }
            }
        } else {
            throw new RuntimeException();
        }
    }
}
