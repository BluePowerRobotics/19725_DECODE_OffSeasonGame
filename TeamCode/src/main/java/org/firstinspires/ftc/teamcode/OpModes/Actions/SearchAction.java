package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class SearchAction implements Action {
    private final Chassis chassis;
    private final Tracker tracker;
    private final Sweeper sweeper;
    private Action trajectoryAction;
    private boolean trajectoryStarted;

    public SearchAction(Chassis chassis, Tracker tracker, Sweeper sweeper) {
        this.chassis = chassis;
        this.tracker = tracker;
        this.sweeper = sweeper;
        this.trajectoryStarted = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 更新追踪状态
        tracker.update();

        // 如果检测到目标，提前中止轨迹，准备去吃球
        Tracker.Target bestTarget = tracker.getBestTarget();
        if (bestTarget != null) {
            sweeper.setStop();
            sweeper.update();
            return false;  // 提前中止
        }

        // 启动清扫器
        sweeper.setEat();
        sweeper.update();

        // 如果轨迹还没开始，构造并启动轨迹
        if (!trajectoryStarted) {
            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
            
            trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose).build();
            //todo: 构造轨迹
            
            trajectoryStarted = true;
        }

        // 执行轨迹
        if (trajectoryAction != null) {
            boolean running = trajectoryAction.run(packet);
            if (!running) {
                // 轨迹执行完毕，返回 true 让 ActionRunner 继续调度
                // 下次调用时会重新构造轨迹
                trajectoryStarted = false;
                trajectoryAction = null;
            }
        }

        // 继续执行（搜索动作永不停止，直到检测到目标或被其他动作取代）
        return true;
    }
}