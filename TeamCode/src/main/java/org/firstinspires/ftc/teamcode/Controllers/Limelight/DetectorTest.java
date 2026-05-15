package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Detector;

@TeleOp(name = "DetectorTest", group = "Tests")
public class DetectorTest extends LinearOpMode {
    private Detector detector;
    
    @Override
    public void runOpMode() {
        // 初始化Detector
        detector = new Detector(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Instructions", "Press play to start detecting purple and green objects");
        telemetry.update();
        String status = "Ready";
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()||gamepad2.aWasPressed()){
                detector.start();
                status="Running";

            } else if (gamepad1.bWasPressed()||gamepad2.bWasPressed()) {
                detector.stop();
                status="Ready2";
            }
            // 检测purple对象
            double[][] purpleCenters = detector.get_center("purple");

            // 检测green对象
            double[][] greenCenters = detector.get_center("green");
            
            // 获取所有检测对象的详细信息
            java.util.List<String> allObjectsInfo = detector.PrintAll();

            // 输出状态信息
            telemetry.addData("Status", status);
            
            // 输出purple对象的中心坐标
            telemetry.addData("Purple Objects", "Count: " + purpleCenters.length);
            for (int i = 0; i < purpleCenters.length; i++) {
                telemetry.addData("Purple " + i, "(%.2f, %.2f)", purpleCenters[i][0], purpleCenters[i][1]);
            }
            
            // 输出green对象的中心坐标
            telemetry.addData("Green Objects", "Count: " + greenCenters.length);
            for (int i = 0; i < greenCenters.length; i++) {
                telemetry.addData("Green " + i, "(%.2f, %.2f)", greenCenters[i][0], greenCenters[i][1]);
            }
            
            // 输出所有检测对象的详细信息
            telemetry.addData("All Objects", "Count: " + (allObjectsInfo.size() > 0 && allObjectsInfo.get(0).equals("No valid Limelight results") ? 0 : allObjectsInfo.size()));
            for (String info : allObjectsInfo) {
                telemetry.addData("Object", info);
            }
            
            telemetry.update();
        }
        
        // 停止Detector
        detector.stop();
    }
}