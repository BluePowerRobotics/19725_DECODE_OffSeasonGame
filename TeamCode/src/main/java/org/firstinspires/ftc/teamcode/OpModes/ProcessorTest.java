package org.firstinspires.ftc.teamcode.OpModes;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;

import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.HypParams;

import java.util.List;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name = "Processor Tester", group = "Test")

public class ProcessorTest extends LinearOpMode {



    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;



    // 调节参数

    private double decimation = 1.0;  // 图像降采样倍数（1.0-8.0，默认2.0）

    private long exposureMs = 30;        // 毫秒

    private double gain = 1;             // 增益 (0 表示自动)

    private double whiteBalanceK = 3200; // 色温 (开尔文)



    private ExposureControl exposureControl;

    private GainControl gainControl;

    private WhiteBalanceControl whiteBalanceControl;



    @Override

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 2. 初始化 VisionPortal（需要硬件映射中的 "Webcam 1"）
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();



        // 3. 等待摄像头进入 STREAMING 状态（重要！）

        telemetry.addData("Waiting for", "STREAMING...");

        telemetry.update();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

            telemetry.addData("Camera State", visionPortal.getCameraState());

            telemetry.update();

            sleep(50); // 避免空转耗尽 CPU
            
        }



        // 4. 安全获取相机控制对象

        try {

            exposureControl = visionPortal.getCameraControl(ExposureControl.class);

        } catch (Exception e) {

            telemetry.addLine("ExposureControl not supported");

        }

        try {

            gainControl = visionPortal.getCameraControl(GainControl.class);

        } catch (Exception e) {

            telemetry.addLine("GainControl not supported");

        }

        try {

            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);

        } catch (Exception e) {

            telemetry.addLine("WhiteBalanceControl not supported");

        }



        // 5. 初始化为手动模式并读取当前真实值（如果支持）

        if (exposureControl != null) {

            try {

                if (exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {

                    exposureControl.setMode(ExposureControl.Mode.Manual);

                }

                exposureMs = exposureControl.getExposure(TimeUnit.MILLISECONDS);

            } catch (Exception e) {

                telemetry.addLine("Exposure init error: " + e.getMessage());

            }

        }

        if (gainControl != null) {

            try {

                gain = gainControl.getGain();

            } catch (Exception e) {

                telemetry.addLine("Gain init error: " + e.getMessage());

            }

        }

        if (whiteBalanceControl != null) {

            try {

                whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);

                whiteBalanceK = whiteBalanceControl.getWhiteBalanceTemperature();

            } catch (Exception e) {

                telemetry.addLine("White Balance init error: " + e.getMessage());

            }

        }



        telemetry.addLine("Ready! Press START");

        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {

            // ---- 手柄调节逻辑 ----

            // 按住任一右扳机则步长减半

            boolean fineAdjust = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);

            double factor = fineAdjust ? 0.5 : 1.0;



            // 手柄1 上下：调节 Decimation（步长 0.1，减半 0.05）

            if (gamepad1.dpadUpWasPressed()) {

                decimation += 0.1 * factor;

            } else if (gamepad1.dpadDownWasPressed()){

                decimation -= 0.1 * factor;

            }

            decimation = Math.max(1.0, Math.min(8.0, decimation));



            // 手柄1 左右：调节 White Balance（步长 100 K，减半 50 K）

            if (gamepad1.dpadRightWasPressed()) {

                whiteBalanceK += 100.0 * factor;

            } else if (gamepad1.dpadLeftWasPressed()) {

                whiteBalanceK -= 100.0 * factor;

            }

            whiteBalanceK = Math.max(2000, Math.min(6500, whiteBalanceK));



            // 手柄2 上下：调节 Exposure（步长 5 ms，减半 2.5 ms）

            if (gamepad2.dpadUpWasPressed()) {

                exposureMs += (long)(5 * factor);

            } else if (gamepad2.dpadDownWasPressed()) {

                exposureMs -= (long)(5 * factor);

            }

            exposureMs = Math.max(0, Math.min(204, exposureMs)); // 0 表示自动



            // 手柄2 左右：调节 Gain（步长 5，减半 2.5）

            if (gamepad2.dpadRightWasPressed()) {

                gain += 5.0 * factor;

            } else if (gamepad2.dpadLeftWasPressed()) {

                gain -= 5.0 * factor;

            }

            gain = Math.max(0, Math.min(255, gain));



            // ---- 将参数应用到硬件 ----

            aprilTag.setDecimation((float) decimation);



            if (exposureControl != null) {

                try {

                    exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);

                } catch (Exception e) {

                    telemetry.addLine("Set exposure failed: " + e.getMessage());

                }

            }

            if (gainControl != null) {

                try {

                    gainControl.setGain((int) gain);

                } catch (Exception e) {

                    telemetry.addLine("Set gain failed: " + e.getMessage());

                }

            }

            if (whiteBalanceControl != null) {

                try {

                    whiteBalanceControl.setWhiteBalanceTemperature((int) whiteBalanceK);

                } catch (Exception e) {

                    telemetry.addLine("Set white balance failed: " + e.getMessage());

                }

            }



            // ---- 获取检测结果并显示 ----

            List<AprilTagDetection> detections = aprilTag.getDetections();



            telemetry.addLine("==== Parameters ====");

            telemetry.addData("Decimation", "%.2f", decimation);

            telemetry.addData("Exposure (ms)", "%d", exposureMs);

            telemetry.addData("Gain", "%.0f", gain);

            telemetry.addData("White Balance (K)", "%.0f", whiteBalanceK);

            telemetry.addData("Fine adjust", fineAdjust);



            telemetry.addLine("\n==== Detections ====");

            telemetry.addData("Count", detections.size());



            if (detections.isEmpty()) {

                telemetry.addLine("No tags detected");

            } else {

                for (AprilTagDetection det : detections) {

                    telemetry.addLine();

                    telemetry.addData("  ID", det.id);

                    if (det.ftcPose != null) {

                        telemetry.addData("    Bearing (deg)", "%.1f", det.ftcPose.bearing);

                        telemetry.addData("    Elevation (deg)", "%.1f", det.ftcPose.elevation);

                        telemetry.addData("    Range (inches)", "%.1f", det.ftcPose.range);

                    } else {

                        telemetry.addLine("    Pose unavailable");

                    }

                }

            }

            telemetry.update();

        }



        // 关闭视觉系统

        visionPortal.close();

    }

}