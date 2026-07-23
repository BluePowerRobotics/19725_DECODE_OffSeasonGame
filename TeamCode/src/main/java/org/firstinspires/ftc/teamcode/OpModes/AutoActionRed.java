package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Autonomous(name = "AutoActionRed", group = "Auto")
public class AutoActionRed extends LinearOpMode {
    private enum Phase {
        GOTO_EAT, EAT, GOTO_START, GOTO_SHOOTING, SHOOT, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.RED;
        int targetTagId = HypParams.targetTagIdRed;

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, HypParams.StartPoseFarRed);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();

        telemetry.addData("Status", "AutoActionRed Initialized");
        telemetry.addData("StartPose", HypParams.StartPoseFarRed);
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.GOTO_EAT;
        boolean parkingStarted = false;

        while (opModeIsActive()) {
            // 检查时间：剩余不足且未开始停车 → 立即清空并启动GoToStopPose
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                sweeper.setStop();
                actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseRed));
                currentPhase = Phase.PARK;
                parkingStarted = true;
            }

            RobotPosition.getInstance().update();
            actionRunner.update();

            // action未完成，继续等待
            if (actionRunner.isBusy()) {
                telemetry.addData("Phase", currentPhase);
                telemetry.addData("Time", "%.1fs", getRuntime());
                telemetry.update();
                continue;
            }

            // 停车完成，退出
            if (currentPhase == Phase.PARK) {
                break;
            }

            // 根据当前phase启动下一个action
            switch (currentPhase) {
                case GOTO_EAT:
                    actionRunner.add(new GoToEatPose(drive, HypParams.EatPoseFarRed));
                    currentPhase = Phase.EAT;
                    break;
                case EAT:
                    // 红色：向+Y方向移动吃球
                    actionRunner.add(new EatAction(drive, sweeper, HypParams.EatDistance, Math.PI / 2, HypParams.EatSecond));
                    currentPhase = Phase.GOTO_START;
                    break;
                case GOTO_START:
                    actionRunner.add(new GoToStartPose(drive, HypParams.StartPoseFarRed));
                    currentPhase = Phase.GOTO_SHOOTING;
                    break;
                case GOTO_SHOOTING:
                    actionRunner.add(new GoToShootingAreaAction(drive, teamColor));
                    currentPhase = Phase.SHOOT;
                    break;
                case SHOOT:
                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                    currentPhase = Phase.PARK;
                    parkingStarted = true;
                    break;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Time", "%.1fs", getRuntime());
            telemetry.update();
        }

        chassis.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime() * 1000 > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}