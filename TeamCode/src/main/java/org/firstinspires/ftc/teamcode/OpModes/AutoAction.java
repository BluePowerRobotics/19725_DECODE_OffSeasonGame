package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToShootingAreaAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SearchAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ShootAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.ChassisController;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;

@Autonomous
@Config
public class AutoAction extends LinearOpMode {
    public enum TEAM_COLOR {
        RED, BLUE
    }

    private TEAM_COLOR teamColor = TEAM_COLOR.RED;
    private boolean initStarted = false;

    private Chassis chassis;
    private Turret turret;
    private Tracker tracker;
    private RobotPosition robotPosition;
    private Sweeper sweeper;
    private double WanderSpeed;
    private int targetTagId;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        WanderSpeed= 1.0;
        while (opModeInInit() || !initStarted) {
            if (gamepad1.a) {
                teamColor = TEAM_COLOR.BLUE;
            }
            if (gamepad1.b) {
                teamColor = TEAM_COLOR.RED;
            }

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

        robotPosition = RobotPosition.RobotPositioninit(hardwareMap, new Pose2d(0, 0, 0));

        ChassisController chassisController = new ChassisController(hardwareMap, telemetry);
        chassis = new Chassis(chassisController, WanderSpeed);

        turret = new Turret(hardwareMap, telemetry, 1.0, 0.0, 0.5);

        sweeper = new Sweeper(hardwareMap, telemetry);

        tracker = new Tracker(hardwareMap, 0.15, 3, 5);
        tracker.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robotPosition.update();

            if (robotPosition.isFull() && !robotPosition.isAbleToShoot()) {
                Actions.runBlocking(new GoToShootingAreaAction(chassis, sweeper));
            }

            if (robotPosition.isAbleToShoot()) {
                chassis.stop();
                Actions.runBlocking(new ShootAction(turret, targetTagId, sweeper));
            }

            if (robotPosition.isEmpty()) {
                Actions.runBlocking(new SearchAction(chassis, tracker, sweeper));
            }
        }

        turret.close();
        tracker.stop();
    }
}