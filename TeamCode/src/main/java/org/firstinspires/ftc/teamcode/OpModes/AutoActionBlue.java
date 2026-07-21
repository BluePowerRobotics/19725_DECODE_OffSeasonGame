package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToEatPose;
import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToShootingAreaAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToStopPose;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ShootAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SearchAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Actions.EatAction;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;



@Autonomous
@Config
public class AutoActionBlue extends LinearOpMode {
    public enum TEAM_COLOR {
        RED, BLUE
    }

    private TEAM_COLOR teamColor = TEAM_COLOR.BLUE;
    private boolean initStarted = false;

    private Chassis chassis;
    private Turret turret;
    private Tracker tracker;
    private Sweeper sweeper;
    private int targetTagId;
    private String lastActionType="Eat";
    private boolean[] eatPoseReached;
    private int currentEatPoseIndex;
    private boolean parked = false;
    private static final long AUTO_TIMEOUT_MS = 30000;
    private static final long PARK_TIME_THRESHOLD_MS = 3000;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while (opModeInInit() || !initStarted) {
            teamColor = TEAM_COLOR.BLUE;
            switch (teamColor) {
                case BLUE:
                    targetTagId = 20;
                    break;
                case RED:
                    targetTagId = 24;
                    break;
            }

            telemetry.addData("TEAM_COLOR", teamColor.toString());
            telemetry.addData("Target Tag ID", targetTagId);
            telemetry.addData("Instructions", "A: Blue, B: Red");
            telemetry.update();

            initStarted = true;
        }

        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor == TEAM_COLOR.RED ?
                TeamColor.RED : TeamColor.BLUE, actionRunner, telemetry, false);

        turret = new Turret(hardwareMap, telemetry);

        sweeper = new Sweeper(hardwareMap, telemetry);

        tracker = new Tracker(hardwareMap);
        tracker.start();

        Pose2d[] eatPoses = (teamColor == TEAM_COLOR.RED) ? HypParams.EatPosesRed : HypParams.EatPosesBlue;
        eatPoseReached = new boolean[eatPoses.length];
        currentEatPoseIndex = 0;

        waitForStart();

        if (isStopRequested()) return;

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();
            actionRunner.update();

            long elapsedTime = System.currentTimeMillis() - startTime;
            long remainingTime = AUTO_TIMEOUT_MS - elapsedTime;

            if (!parked && remainingTime <= PARK_TIME_THRESHOLD_MS) {
                actionRunner.add(new GoToStopPose(chassis, sweeper, HypParams.StopPoseBlue));
                lastActionType = "GoToStopPose";
                parked = true;
                continue;
            }

            if(!actionRunner.isBusy()) {
                // ???????? EatAction????? GoToShootingAreaAction
                if ((lastActionType.equals("Eat")&&!RobotPosition.getInstance().isEmpty() && !RobotPosition.getInstance().isAbleToShoot())||(RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
                    actionRunner.add(new GoToShootingAreaAction(chassis, sweeper));
                    lastActionType = "GoToShootingArea";
                }
                else if (!RobotPosition.getInstance().isEmpty() && RobotPosition.getInstance().isAbleToShoot()) {
                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                    lastActionType = "Shoot";
                }
                else if (RobotPosition.getInstance().isEmpty() || (!RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
                    // ?????????????
                    boolean hasUnreachedPose = false;
                    for (int i = 0; i < eatPoseReached.length; i++) {
                        if (!eatPoseReached[i]) {
                            hasUnreachedPose = true;
                            break;
                        }
                    }

                    if (hasUnreachedPose) {
                        // ?????????????????
                        if (lastActionType.equals("GoToEatPose")) {
                            // ????????????
                            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
                            Pose2d targetPose = eatPoses[currentEatPoseIndex];
                            double distance = Math.hypot(
                                    currentPose.position.x - targetPose.position.x,
                                    currentPose.position.y - targetPose.position.y
                            );
                            // ????????(5??)?????GoToEatPose
                            if (distance > 5.0) {
                                actionRunner.add(new GoToEatPose(chassis, sweeper, targetPose));
                                lastActionType = "GoToEatPose";
                            } else {
                                eatPoseReached[currentEatPoseIndex] = true;
                                // ??????????????????
                                if (tracker.getHasTarget()) {
                                    actionRunner.add(new EatAction(chassis, tracker, sweeper));
                                    lastActionType = "Eat";
                                } else {
                                    // ?????????????????
                                    for (int i = 0; i < eatPoseReached.length; i++) {
                                        if (!eatPoseReached[i]) {
                                            currentEatPoseIndex = i;
                                            break;
                                        }
                                    }
                                    actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
                                    lastActionType = "GoToEatPose";
                                }
                            }
                        } else {
                            // ?????????????
                            for (int i = 0; i < eatPoseReached.length; i++) {
                                if (!eatPoseReached[i]) {
                                    currentEatPoseIndex = i;
                                    break;
                                }
                            }
                            actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
                            lastActionType = "GoToEatPose";
                        }
                    } else {
                        // ???????????????? SearchAction
                        if(tracker.getHasTarget()){
                            actionRunner.add(new EatAction(chassis, tracker, sweeper));
                            lastActionType = "Eat";
                        }
                        else{
                            actionRunner.add(new SearchAction(chassis, tracker, sweeper,
                                    teamColor == TEAM_COLOR.RED ? TeamColor.RED : TeamColor.BLUE));
                            lastActionType = "Search";
                        }
                    }
                }
            }
        }
    }
}