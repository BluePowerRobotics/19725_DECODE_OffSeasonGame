package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.utility.HypParams.EatPoseFarBlue;
import static org.firstinspires.ftc.teamcode.utility.HypParams.EatPoseFarRed;
import static org.firstinspires.ftc.teamcode.utility.HypParams.searchPoseBlue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Detector;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret_2;
import org.firstinspires.ftc.teamcode.OpModes.Actions.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous(name = "AutoActionFar_Blue_1", group = "Auto")
public class AutoActionFar_Blue_1 extends LinearOpMode {
    private enum Phase {
        GOTO_SHOOTING, SHOOT, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private Turret_2 turret_2;
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
        turret_2 = new Turret_2(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();

        telemetry.addData("Status", "AutoActionFar_RED Initialized");
        telemetry.addData("StartPose", HypParams.StartPoseFarRed);
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.GOTO_SHOOTING;
        boolean parkingStarted = false;
        boolean eattunnel = false;

        while (opModeIsActive()) {
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                sweeper.setStop();
                sweeper.update();
                actionRunner.add(new GoToStopPose_Far(drive, HypParams.StopPoseFarRed, turret_2));
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
                case GOTO_SHOOTING:
                    actionRunner.add(new GoToShootingFarAreaAction(drive, teamColor));
                    currentPhase = Phase.SHOOT;
                    break;
                case SHOOT:
                    actionRunner.add(new ShootAction_Far(chassis, turret_2, targetTagId, sweeper));
                    currentPhase = Phase.PARK;
                    parkingStarted = true;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Time", "%.1fs", getRuntime());
            telemetry.update();
        }

        chassis.stop();
        sweeper.setStop();
        sweeper.update();
        turret_2.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime() * 1000 > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}