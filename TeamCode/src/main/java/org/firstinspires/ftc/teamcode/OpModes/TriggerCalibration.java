package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.HypParams;

@Config
@TeleOp(name = "Trigger Calibration", group = "Tests")
public class TriggerCalibration extends LinearOpMode {

    private Servo triggerServo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        triggerServo = hardwareMap.get(Servo.class, "trigger");

        // 从 HypParams 读取初始位置
        double currentPosition = HypParams.triggerResetPosition;
        triggerServo.setPosition(currentPosition);

        telemetry.addLine("=== Trigger Servo Calibration ===");
        telemetry.addData("Instructions", "Left stick Y to move servo (up=increase, down=decrease)");
        telemetry.addData("A", "Save current as Launch Position");
        telemetry.addData("B", "Save current as Reset Position");
        telemetry.addData("X", "Move to Launch Position");
        telemetry.addData("Y", "Move to Reset Position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 左摇杆Y轴控制舵机位置（上推增加，下拉减少）
            double stickInput = -gamepad1.left_stick_y;
            if (Math.abs(stickInput) > 0.05) {
                currentPosition += stickInput * 0.005;
                currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));
                triggerServo.setPosition(currentPosition);
            }

            // A键保存当前为发射位置
            if (gamepad1.a) {
                HypParams.triggerLaunchPosition = currentPosition;
                telemetry.addData(">>>", "Launch position saved: %.3f", currentPosition);
            }

            // B键保存当前为复位位置
            if (gamepad1.b) {
                HypParams.triggerResetPosition = currentPosition;
                telemetry.addData(">>>", "Reset position saved: %.3f", currentPosition);
            }

            // X键跳转到发射位置
            if (gamepad1.x) {
                currentPosition = HypParams.triggerLaunchPosition;
                triggerServo.setPosition(currentPosition);
            }

            // Y键跳转到复位位置
            if (gamepad1.y) {
                currentPosition = HypParams.triggerResetPosition;
                triggerServo.setPosition(currentPosition);
            }

            // 显示信息
            telemetry.addData("Current Position", "%.3f", currentPosition);
            telemetry.addData("Launch Position", "%.3f", HypParams.triggerLaunchPosition);
            telemetry.addData("Reset Position", "%.3f", HypParams.triggerResetPosition);
            telemetry.addLine();
            telemetry.addData("A to save Launch", "B to save Reset");
            telemetry.addData("X to goto Launch", "Y to goto Reset");
            telemetry.update();
        }
    }
}