package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Detector;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Actions.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous(name = "AutoActionRed", group = "Auto")
public class AutoActionRed extends LinearOpMode {
    private enum Phase {
        SHOOT, SEARCH, EAT, RETURN_TO_START, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private Detector detector;
    private ActionRunner actionRunner;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.RED;
        int targetTagId = HypParams.targetTagIdRed;

        actionRunner = new ActionRunner();
        detector = new Detector(hardwareMap);
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, false);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();

        detector.start();

        telemetry.addData("Status", "AutoActionRed Initialized");
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.SHOOT;
        boolean parkingStarted = false;

        // 主循环：检查时间 → 检查isBusy → 状态转移
        while (opModeIsActive()) {
            // 检查时间：不足且未开始停车 → 立即清空并启动GoToStopPose
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseRed));
                currentPhase = Phase.PARK;
                parkingStarted = true;
            }

            RobotPosition.getInstance().update();
            actionRunner.update();

            // action未完成，继续等待
            if (actionRunner.isBusy()) {
                telemetry.addData("Phase", currentPhase);
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
                case SHOOT:
                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                    currentPhase = Phase.SEARCH;
                    break;
                case SEARCH:
                    actionRunner.add(new SearchAction(drive, HypParams.searchPoseRed, detector));
                    currentPhase = Phase.EAT;
                    break;
                case EAT:
                    actionRunner.add(new EatAction(drive, sweeper, HypParams.EatDistance, HypParams.EatSecond));
                    currentPhase = Phase.RETURN_TO_START;
                    break;
                case RETURN_TO_START:
                    actionRunner.add(new GoToStartPose(drive, HypParams.startPoseRed));
                    currentPhase = Phase.SHOOT;
                    break;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Time", "%.1fs", getRuntime().seconds());
            telemetry.update();
        }

        detector.stop();
        chassis.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime().milliseconds() > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}