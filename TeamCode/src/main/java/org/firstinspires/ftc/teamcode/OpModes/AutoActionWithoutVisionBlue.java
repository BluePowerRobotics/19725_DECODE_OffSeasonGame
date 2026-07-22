package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Actions.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous(name = "AutoActionWithoutVisionBlue", group = "Auto")
public class AutoActionWithoutVisionBlue extends LinearOpMode {
    private enum Phase {
        SHOOT_PRELOAD, GOTO_EAT, EAT, GOTO_SHOOTING, SHOOT, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.BLUE;
        int targetTagId = HypParams.targetTagIdBlue;
        Pose2d[] eatPoses = HypParams.EatPosesBlue;
        double eatDistance = -HypParams.EatDistance; // 蓝色向-Y移动

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, false);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();

        telemetry.addData("Status", "AutoActionWithoutVisionBlue Initialized");
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.SHOOT_PRELOAD;
        int eatPoseIndex = 0;
        boolean parkingStarted = false;

        // 主循环：检查时间 → 检查isBusy → 状态转移
        while (opModeIsActive()) {
            // 检查时间：不足且未开始停车 → 立即清空并启动GoToStopPose
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseBlue));
                currentPhase = Phase.PARK;
                parkingStarted = true;
            }

            RobotPosition.getInstance().update();
            actionRunner.update();

            // action未完成，继续等待
            if (actionRunner.isBusy()) {
                telemetry.addData("Phase", currentPhase);
                telemetry.addData("EatIndex", "%d/%d", eatPoseIndex, eatPoses.length);
                telemetry.addData("Time", "%.1fs", getRuntime().seconds());
                telemetry.update();
                continue;
            }

            // 停车完成，退出
            if (currentPhase == Phase.PARK) {
                break;
            }

            // 根据当前phase启动下一个action
            switch (currentPhase) {
                case SHOOT_PRELOAD:
                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                    currentPhase = Phase.GOTO_EAT;
                    break;
                case GOTO_EAT:
                    actionRunner.add(new GoToEatPose(drive, eatPoses[eatPoseIndex]));
                    currentPhase = Phase.EAT;
                    break;
                case EAT:
                    actionRunner.add(new EatAction(drive, sweeper, eatDistance, HypParams.EatSecond));
                    currentPhase = Phase.GOTO_SHOOTING;
                    break;
                case GOTO_SHOOTING:
                    actionRunner.add(new GoToShootingAreaAction(drive));
                    currentPhase = Phase.SHOOT;
                    break;
                case SHOOT:
                    eatPoseIndex++;
                    if (eatPoseIndex >= eatPoses.length) {
                        // 所有吃球位姿用完，停车
                        actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseBlue));
                        currentPhase = Phase.PARK;
                        parkingStarted = true;
                    } else {
                        actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                        currentPhase = Phase.GOTO_EAT;
                    }
                    break;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("EatIndex", "%d/%d", eatPoseIndex, eatPoses.length);
            telemetry.addData("Time", "%.1fs", getRuntime().seconds());
            telemetry.update();
        }

        chassis.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime().milliseconds() > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}