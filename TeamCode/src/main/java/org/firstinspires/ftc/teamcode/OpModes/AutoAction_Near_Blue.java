package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Actions_Near.GoToEatPose_Near;
import org.firstinspires.ftc.teamcode.OpModes.Actions_Near.GoToShootingAreaAction_Near;
import org.firstinspires.ftc.teamcode.OpModes.Actions_Near.ShootAction_Near;
import org.firstinspires.ftc.teamcode.OpModes.Actions_Near.EatAction_Near;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous
@Config
public class AutoAction_Near_Blue extends LinearOpMode {
    private Chassis chassis;
    private Turret turret;
    private Tracker tracker;
    private Sweeper sweeper;
    private static final int targetTagId = 20;
    private String lastActionType = "Init";
    private boolean isEmpty = true;
    private boolean[] eatPoseReached;
    private int currentEatPoseIndex;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, TeamColor.BLUE, actionRunner, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        tracker = new Tracker(hardwareMap);
        tracker.start();

        Pose2d[] eatPoses = HypParams.EatPosesBlue;
        eatPoseReached = new boolean[eatPoses.length];
        currentEatPoseIndex = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();
            actionRunner.update();

            if (tracker.getHasTarget()) {
                isEmpty = false;
            }

            if (!actionRunner.isBusy()) {
                if (lastActionType.equals("Shoot")) {
                    isEmpty = true;
                }

                if (lastActionType.equals("Eat")) {
                    isEmpty = false;
                }

                if (!isEmpty && isInNearShootingArea()) {
                    actionRunner.add(new ShootAction_Near(chassis, turret, targetTagId, sweeper));
                    lastActionType = "Shoot";
                } else if (!isEmpty && !isInNearShootingArea()) {
                    actionRunner.add(new GoToShootingAreaAction_Near(chassis, sweeper, false));
                    lastActionType = "GoToShootingArea";
                } else {
                    boolean hasUnreachedPose = false;
                    for (int i = 0; i < eatPoseReached.length; i++) {
                        if (!eatPoseReached[i]) {
                            hasUnreachedPose = true;
                            break;
                        }
                    }

                    if (hasUnreachedPose) {
                        if (lastActionType.equals("GoToEatPose")) {
                            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
                            Pose2d targetPose = eatPoses[currentEatPoseIndex];
                            double distance = Math.hypot(
                                    currentPose.position.x - targetPose.position.x,
                                    currentPose.position.y - targetPose.position.y
                            );
                            if (distance > 5.0) {
                                actionRunner.add(new GoToEatPose_Near(chassis, sweeper, targetPose));
                                lastActionType = "GoToEatPose";
                            } else {
                                eatPoseReached[currentEatPoseIndex] = true;
                                if (tracker.getHasTarget()) {
                                    actionRunner.add(new EatAction_Near(chassis, tracker, sweeper));
                                    lastActionType = "Eat";
                                } else {
                                    int nextIndex = findNextUnreachedEatPose();
                                    if (nextIndex != -1) {
                                        currentEatPoseIndex = nextIndex;
                                        actionRunner.add(new GoToEatPose_Near(chassis, sweeper, eatPoses[currentEatPoseIndex]));
                                        lastActionType = "GoToEatPose";
                                    }
                                }
                            }
                        } else {
                            int nextIndex = findNextUnreachedEatPose();
                            if (nextIndex != -1) {
                                currentEatPoseIndex = nextIndex;
                                actionRunner.add(new GoToEatPose_Near(chassis, sweeper, eatPoses[currentEatPoseIndex]));
                                lastActionType = "GoToEatPose";
                            }
                        }
                    } else {
                        if (!isEmpty) {
                            if (isInNearShootingArea()) {
                                actionRunner.add(new ShootAction_Near(chassis, turret, targetTagId, sweeper));
                                lastActionType = "Shoot";
                            } else {
                                actionRunner.add(new GoToShootingAreaAction_Near(chassis, sweeper, false));
                                lastActionType = "GoToShootingArea";
                            }
                        } else {
                            break;
                        }
                    }
                }
            }
        }
    }

    private boolean isInNearShootingArea() {
        return HypParams.BoundingBox.inAbsolute(RobotPosition.getInstance().getPose2d())
                .IsIntersected(HypParams.SHOOTING_AREA_NEAR_BLUE);
    }

    private int findNextUnreachedEatPose() {
        for (int i = 0; i < eatPoseReached.length; i++) {
            if (!eatPoseReached[i]) {
                return i;
            }
        }
        return -1;
    }
}