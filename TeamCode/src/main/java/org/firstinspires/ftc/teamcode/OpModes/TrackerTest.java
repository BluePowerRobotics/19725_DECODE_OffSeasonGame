package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;

import java.util.List;

@TeleOp(name = "TrackerTest", group = "Tests")
public class TrackerTest extends LinearOpMode {
    private Tracker tracker;
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL = 1000; // 1秒更新一次
    
    @Override
    public void runOpMode() {
        // 初始化Tracker
        tracker = new Tracker(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press play to start tracking targets");
        telemetry.update();
        
        waitForStart();
        
        // 启动追踪器
        tracker.start();
        
        while (opModeIsActive()) {
            // 更新追踪状态
            tracker.update();
            
            // 每秒更新一次显示
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
                lastUpdateTime = currentTime;
                
                // 获取最优目标
                Tracker.Target bestTarget = tracker.getBestTarget();
                
                // 输出最优目标信息
                telemetry.addData("Status", "Running");
                if (bestTarget != null) {
                    double distance = bestTarget.getDistanceToCamera();
                    telemetry.addData("Best Target", "ID: " + bestTarget.id);
                    telemetry.addData("Best Target", "Color: " + bestTarget.color);
                    telemetry.addData("Best Target", "Frames: " + bestTarget.consecutiveFrames);
                    telemetry.addData("Best Target", "Coord: (%.2f, %.2f)", bestTarget.centerX, bestTarget.centerY);
                    telemetry.addData("Best Target", "Distance: %.2f m", distance);
                } else {
                    telemetry.addData("Best Target", "None");
                }
                
                // 输出所有目标信息
                List<Tracker.Target> allTargets = tracker.getTargets();
                telemetry.addData("All Targets", "Count: " + allTargets.size());
                
                for (Tracker.Target target : allTargets) {
                    if (target == bestTarget) continue; // 跳过已经显示的最优目标
                    
                    double distance = target.getDistanceToCamera();
                    telemetry.addData("Target " + target.id, "Color: " + target.color);
                    telemetry.addData("Target " + target.id, "Frames: " + target.consecutiveFrames);
                    telemetry.addData("Target " + target.id, "Coord: (%.2f, %.2f)", target.centerX, target.centerY);
                    telemetry.addData("Target " + target.id, "Distance: %.2f m", distance);
                }
                
                telemetry.update();
            }
        }
        
        // 停止追踪器
        tracker.stop();
    }
}
