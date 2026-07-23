package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Actions.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous(name = "AutoActionFar_Red", group = "Auto")
public class AutoActionFar_Red extends LinearOpMode {
    private enum Phase {
        GOTO_SHOOTING_PRELOAD, SHOOT_PRELOAD, GOTO_EAT_FAR, EAT_FAR, WAIT_5S, GOTO_TUNNEL, LIMELIGHT_EAT, GOTO_SHOOTING, SHOOT, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;
    private MecanumDrive drive;
    private Tracker tracker;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.RED;
        int targetTagId = HypParams.targetTagIdRed;
        Pose2d[] eatPoses = HypParams.EatPoseRed_FAR;

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, HypParams.StartPoseFarRed);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();
        tracker = new Tracker(hardwareMap);

        telemetry.addData("Status", "AutoActionFar_Red Initialized");
        telemetry.addData("StartPose", HypParams.StartPoseFarRed);
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.GOTO_SHOOTING_PRELOAD;
        int eatPoseIndex = 0;
        int eatPoseIndex_preload = 0;
        boolean parkingStarted = false;

        while (opModeIsActive()) {
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                sweeper.setStop();
                sweeper.update();
                actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseFarRed));
                currentPhase = Phase.PARK;
                parkingStarted = true;
            }

            RobotPosition.getInstance().update();
            actionRunner.update();

            if (actionRunner.isBusy()) {
                telemetry.addData("Phase", currentPhase);
                telemetry.addData("Time", "%.1fs", getRuntime());
                telemetry.update();
                continue;
            }

            if (currentPhase == Phase.PARK) {
                break;
            }

            switch (currentPhase) {
                case GOTO_SHOOTING_PRELOAD:
                    actionRunner.add(new GoToShootingFarAreaAction(drive, teamColor));
                    currentPhase = Phase.SHOOT_PRELOAD;
                    break;
                case SHOOT_PRELOAD:
                    if (eatPoseIndex_preload >= eatPoses.length) {
                        actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                        currentPhase = Phase.WAIT_5S;
                        break;
                    }else {
                        actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                        currentPhase = Phase.GOTO_EAT_FAR;
                        break;
                    }
                case GOTO_EAT_FAR:
                    actionRunner.add(new GoToEatPose(drive, eatPoses[eatPoseIndex]));
                    currentPhase = Phase.EAT_FAR;
                    break;
                case EAT_FAR:
                    eatPoseIndex_preload++;
                    actionRunner.add(new EatAction(drive, sweeper, HypParams.EatDistance, Math.PI / 2, HypParams.EatSecond));
                    currentPhase = Phase.GOTO_SHOOTING_PRELOAD;
                    break;
                case WAIT_5S:
                    actionRunner.add(new WaitAction(HypParams.AUTO_WAIT_DURATION_MS));
                    currentPhase = Phase.GOTO_TUNNEL;
                    break;
                case GOTO_TUNNEL:
                    actionRunner.add(new GoToEatPose(drive, HypParams.TunnelPoseRed));
                    currentPhase = Phase.LIMELIGHT_EAT;
                    break;
                case LIMELIGHT_EAT:
                    actionRunner.add(new LimelightEatAction(drive, sweeper, tracker, HypParams.LIMELIGHT_EAT_DURATION_MS));
                    currentPhase = Phase.GOTO_SHOOTING;
                    parkingStarted = true;
                    break;
                case GOTO_SHOOTING:
                    actionRunner.add(new GoToShootingFarAreaAction(drive, teamColor));
                    currentPhase = Phase.SHOOT;
                    break;
                case SHOOT:
                    eatPoseIndex++;
                    if (eatPoseIndex >= eatPoses.length) {
                        // 所有吃球位姿用完，停车
                        actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseFarRed));
                        currentPhase = Phase.PARK;
                        parkingStarted = true;
                    } else {
                        actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                        currentPhase = Phase.GOTO_EAT_FAR;
                    }
                    break;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Time", "%.1fs", getRuntime());
            telemetry.update();
        }

        chassis.stop();
        sweeper.setStop();
        sweeper.update();
        turret.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime() * 1000 > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}