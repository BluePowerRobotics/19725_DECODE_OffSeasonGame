package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Actions.GoToShootingAreaAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ShootAction;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SearchAction;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.OpModes.Actions.EatAction;



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
    private Sweeper sweeper;
    private int targetTagId;
    private String lastActionType;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor == TEAM_COLOR.RED ?
            Chassis.TEAM_COLOR.RED : Chassis.TEAM_COLOR.BLUE, actionRunner, telemetry);

        turret = new Turret(hardwareMap, telemetry);

        sweeper = new Sweeper(hardwareMap, telemetry);

        tracker = new Tracker(hardwareMap);
        tracker.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();
            actionRunner.update();
            if(!actionRunner.isBusy()) {
                // 如果上一个动作是 EatAction，直接进入 GoToShootingAreaAction
                if ((lastActionType.equals("Eat")&&!RobotPosition.getInstance().isEmpty() && !RobotPosition.getInstance().isAbleToShoot())||(RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
                    actionRunner.add(new GoToShootingAreaAction(chassis, sweeper));
                    lastActionType = "GoToShootingArea";
                }
                else if (!RobotPosition.getInstance().isEmpty() && RobotPosition.getInstance().isAbleToShoot()) {
                    actionRunner.add(new ShootAction(chassis, turret, targetTagId, sweeper));
                    lastActionType = "Shoot";
                }
                else if (RobotPosition.getInstance().isEmpty() || (!RobotPosition.getInstance().isFull() && !RobotPosition.getInstance().isAbleToShoot())) {
                    if(tracker.getHasTarget()){
                        actionRunner.add(new EatAction(chassis, tracker, sweeper));
                        lastActionType = "Eat";
                    }
                    else{
                        actionRunner.add(new SearchAction(chassis, tracker, sweeper, 
                            teamColor == TEAM_COLOR.RED ? Chassis.TEAM_COLOR.RED : Chassis.TEAM_COLOR.BLUE));
                        lastActionType = "Search";
                    }
                }
            }
        }
    }
}