package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@TeleOp(name = "SensitivityTest", group = "Tests")
public class SensitivityTest extends LinearOpMode {
    private double currentMaxV = HypParams.maxV;
    private double currentMaxOmega = HypParams.maxOmega;
    private static final double V_STEP = 0.1;
    private static final double OMEGA_STEP = 0.05;
    private boolean useNoHeadMode = false;
    private long lastNanoTime = 0;
    private Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, TeamColor.RED, actionRunner, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            if (gamepad1.dpadUpWasPressed()) {
                currentMaxV += V_STEP;
                HypParams.maxV = currentMaxV;
            }
            if (gamepad1.dpadDownWasPressed()) {
                currentMaxV = Math.max(0.1, currentMaxV - V_STEP);
                HypParams.maxV = currentMaxV;
            }
            if (gamepad1.dpadRightWasPressed()) {
                currentMaxOmega += OMEGA_STEP;
                HypParams.maxOmega = currentMaxOmega;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                currentMaxOmega = Math.max(0.1, currentMaxOmega - OMEGA_STEP);
                HypParams.maxOmega = currentMaxOmega;
            }

            if (gamepad1.xWasReleased()) {
                useNoHeadMode = !useNoHeadMode;
            }

            MecanumDrive drive = RobotPosition.getInstance().getDrive();
            double Kx = gamepad1.left_stick_x;
            double Ky = gamepad1.left_stick_y;
            double Komega = gamepad1.right_stick_x;
            double omega = Komega * currentMaxOmega;

            if (useNoHeadMode) {
                double fieldX = Ky;
                double fieldY = Kx;
                double angle = -RobotPosition.getInstance().getTheta();
                double cosAngle = Math.cos(angle);
                double sinAngle = Math.sin(angle);
                double robotX = (fieldX * cosAngle - fieldY * sinAngle) * currentMaxV;
                double robotY = (fieldX * sinAngle + fieldY * cosAngle) * currentMaxV;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(robotX, robotY), omega));
            } else {
                double forward = Ky * currentMaxV;
                double strafe = Kx * currentMaxV;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), omega));
            }

            telemetry.addData("Current maxV", "%.2f in/s", currentMaxV);
            telemetry.addData("Current maxOmega", "%.2f rad/s (%.1f deg/s)", currentMaxOmega, Math.toDegrees(currentMaxOmega));
            telemetry.addData("HypParams maxV", "%.2f", HypParams.maxV);
            telemetry.addData("HypParams maxOmega", "%.2f", HypParams.maxOmega);
            telemetry.addData("NoHeadMode", useNoHeadMode);
            telemetry.addData("LeftStickX", "%.2f", Kx);
            telemetry.addData("LeftStickY", "%.2f", Ky);
            telemetry.addData("RightStickX", "%.2f", Komega);
            telemetry.addData("Instructions", "D-pad Up/Down: maxV(+/-%.1f) | Left/Right: maxOmega(+/-%.2f) | X: NoHead", V_STEP, OMEGA_STEP);
            chassis.telemetry();
            telemetry.addData("FPS", 1000000000.0 / (System.nanoTime() - lastNanoTime));
            lastNanoTime = System.nanoTime();
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        chassis.stop();
    }
}