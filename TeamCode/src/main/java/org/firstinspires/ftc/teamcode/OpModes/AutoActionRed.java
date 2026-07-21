//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToEatPose;
//import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToShootingAreaAction;
//import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToStopPose;
//import org.firstinspires.ftc.teamcode.OpModes.Actions.ShootAction;
//import org.firstinspires.ftc.teamcode.OpModes.Actions.SearchAction;
//import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
//import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
//import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
//import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
//import org.firstinspires.ftc.teamcode.utility.ActionRunner;
//import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
//import org.firstinspires.ftc.teamcode.OpModes.Actions.EatAction;
//import org.firstinspires.ftc.teamcode.utility.HypParams;
//import org.firstinspires.ftc.teamcode.utility.TeamColor;
//
//
//
//@Autonomous
//@Config
//public class AutoActionRed extends LinearOpMode {
//    public enum TEAM_COLOR {
//        RED, BLUE
//    }
//
//    private TEAM_COLOR teamColor = TEAM_COLOR.RED;
//    private boolean initStarted = false;
//
//    private Chassis chassis;
//    private Turret turret;
//    private Tracker tracker;
//    private Sweeper sweeper;
//    private int targetTagId;
//    private String lastActionType="Eat";
//    private boolean[] eatPoseReached;
//    private int currentEatPoseIndex;
//    private boolean parked = false;
//    private static final long AUTO_TIMEOUT_MS = 30000;
//    private static final long PARK_TIME_THRESHOLD_MS = 3000;
//
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        while (opModeInInit() || !initStarted) {
//            teamColor = TEAM_COLOR.RED;
//            switch (teamColor) {
//                case BLUE:
//                    targetTagId = 20;
//                    break;
//                case RED:
//                    targetTagId = 24;
//                    break;
//            }
//
//            telemetry.addData("TEAM_COLOR", teamColor.toString());
//            telemetry.addData("Target Tag ID", targetTagId);
//            telemetry.addData("Instructions", "A: Blue, B: Red");
//            telemetry.update();
//
//            initStarted = true;
//        }
//
//        ActionRunner actionRunner = new ActionRunner();
//        chassis = new Chassis(hardwareMap, teamColor == TEAM_COLOR.RED ?
//            TeamColor.RED : TeamColor.BLUE, actionRunner, telemetry);
//
//        turret = new Turret(hardwareMap, telemetry);
//
//        sweeper = new Sweeper(hardwareMap, telemetry);
//
//        tracker = new Tracker(hardwareMap);
//        tracker.start();
//
//        Pose2d[] eatPoses = (teamColor == TEAM_COLOR.RED) ? HypParams.EatPosesRed : HypParams.EatPosesBlue;
//        eatPoseReached = new boolean[eatPoses.length];
//        currentEatPoseIndex = 0;
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        long startTime = System.currentTimeMillis();
//
//        while (opModeIsActive()) {
//            RobotPosition.getInstance().update();
//            actionRunner.update();
//
//            long elapsedTime = System.currentTimeMillis() - startTime;
//            long remainingTime = AUTO_TIMEOUT_MS - elapsedTime;
//
//            if (!parked && remainingTime <= PARK_TIME_THRESHOLD_MS) {
//                actionRunner.add(new GoToStopPose(chassis, sweeper, HypParams.StopPoseRed));
//                lastActionType = "GoToStopPose";
//                parked = true;
//                continue;
//            }
//
//            if(!actionRunner.isBusy()) {
//                // 如果上一个动作是 EatAction，直接进入 GoToShootingAreaAction
//                if ((lastActionType.equals("Eat")&&!RobotPosition.getInstance().isEmpty() && !RobotPosition.getInstance().isAbleToShoot())||(RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
//                    actionRunner.add(new GoToShootingAreaAction(chassis, sweeper));
//                    lastActionType = "GoToShootingArea";
//                }
//                else if (!RobotPosition.getInstance().isEmpty() && RobotPosition.getInstance().isAbleToShoot()) {
//                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
//                    lastActionType = "Shoot";
//                }
//                else if (RobotPosition.getInstance().isEmpty() || (!RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
//                    // 检查是否有未到达的吃球位姿
//                    boolean hasUnreachedPose = false;
//                    for (int i = 0; i < eatPoseReached.length; i++) {
//                        if (!eatPoseReached[i]) {
//                            hasUnreachedPose = true;
//                            break;
//                        }
//                    }
//
//                    if (hasUnreachedPose) {
//                        // 有未到达的吃球位姿，先移动到该位姿
//                        if (lastActionType.equals("GoToEatPose")) {
//                            // 检查是否真正到达目标位置
//                            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
//                            Pose2d targetPose = eatPoses[currentEatPoseIndex];
//                            double distance = Math.hypot(
//                                currentPose.position.x - targetPose.position.x,
//                                currentPose.position.y - targetPose.position.y
//                            );
//                            // 如果距离大于阈值(5英寸)，重新执行GoToEatPose
//                            if (distance > 5.0) {
//                                actionRunner.add(new GoToEatPose(chassis, sweeper, targetPose));
//                                lastActionType = "GoToEatPose";
//                            } else {
//                                eatPoseReached[currentEatPoseIndex] = true;
//                                // 已经到达吃球位姿，检查视野内是否有球
//                                if (tracker.getHasTarget()) {
//                                    actionRunner.add(new EatAction(chassis, tracker, sweeper));
//                                    lastActionType = "Eat";
//                                } else {
//                                    // 没有球，前往下一个未到达的吃球位姿
//                                    for (int i = 0; i < eatPoseReached.length; i++) {
//                                        if (!eatPoseReached[i]) {
//                                            currentEatPoseIndex = i;
//                                            break;
//                                        }
//                                    }
//                                    actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
//                                    lastActionType = "GoToEatPose";
//                                }
//                            }
//                        } else {
//                            // 前往第一个未到达的吃球位姿
//                            for (int i = 0; i < eatPoseReached.length; i++) {
//                                if (!eatPoseReached[i]) {
//                                    currentEatPoseIndex = i;
//                                    break;
//                                }
//                            }
//                            actionRunner.add(new GoToEatPose(chassis, sweeper, eatPoses[currentEatPoseIndex]));
//                            lastActionType = "GoToEatPose";
//                        }
//                    } else {
//                        // 所有吃球位姿都已到达，执行原来的 SearchAction
//                        if(tracker.getHasTarget()){
//                            actionRunner.add(new EatAction(chassis, tracker, sweeper));
//                            lastActionType = "Eat";
//                        }
//                        else{
//                            actionRunner.add(new SearchAction(chassis, tracker, sweeper,
//                                teamColor == TEAM_COLOR.RED ? TeamColor.RED : TeamColor.BLUE));
//                            lastActionType = "Search";
//                        }
//                    }
//                }
//            }
//        }
//    }
//}