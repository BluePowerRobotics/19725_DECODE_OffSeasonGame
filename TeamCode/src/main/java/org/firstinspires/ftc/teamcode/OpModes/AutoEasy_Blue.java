package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Autonomous(name = "AutoEasy_Blue")
public class AutoEasy_Blue extends LinearOpMode {
    private Action trajectoryAction;
    private boolean trajectoryStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ActionRunner actionRunner = new ActionRunner();
        Chassis chassis = new Chassis(hardwareMap, TeamColor.BLUE, actionRunner, telemetry);

        Pose2d targetPose = new Pose2d(60, -60, 0);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            if (!trajectoryStarted) {
                Pose2d currentPose = RobotPosition.getInstance().getPose2d();
                trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                        .splineToLinearHeading(targetPose, currentPose.heading.toDouble())
                        .build();
                trajectoryStarted = true;
            }

            TelemetryPacket packet = new TelemetryPacket();
            if (trajectoryAction != null && trajectoryAction.run(packet)) {
                // 轨迹执行中
            } else {
                chassis.stop();
                break;
            }

            telemetry.addData("Target", targetPose.toString());
            telemetry.addData("Current", RobotPosition.getInstance().getPose2d().toString());
            telemetry.update();
        }
    }
}