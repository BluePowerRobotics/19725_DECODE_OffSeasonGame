package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
    private final Chassis.TEAM_COLOR teamColor;
    private Action trajectoryAction;
    private boolean trajectoryStarted;

    public SearchAction(Chassis chassis, Tracker tracker, Sweeper sweeper, Chassis.TEAM_COLOR teamColor) {
        this.chassis = chassis;
        this.tracker = tracker;
        this.sweeper = sweeper;
        this.teamColor = teamColor;
        this.trajectoryStarted = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 更新追踪状态
        tracker.update();

        // 如果检测到目标，提前中止轨迹，准备去吃球
        if (tracker.getHasTarget()) {
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
            
            // 根据队伍颜色选择不同的搜索轨迹
            if (teamColor == Chassis.TEAM_COLOR.BLUE) {
                // 蓝队搜索轨迹：转向0° → 平移至 y=-50 → 前进至 x=-10 → 后退至 x=-60
                trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                    .turn(Math.PI)
                    .strafeTo(new Vector2d(currentPose.position.x, 50))
                    .strafeTo(new Vector2d(10, 50))
                    .strafeTo(new Vector2d(60, 50))
                    .build();
            } else {
                // 红队搜索轨迹：转向0° → 平移至 y=50 → 前进至 x=-10 → 后退至 x=-60
                trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                    .turn(Math.PI)
                    .strafeTo(new Vector2d(currentPose.position.x, -50))
                    .strafeTo(new Vector2d(10, -50))
                    .strafeTo(new Vector2d(60, -50))
                    .build();   
            }
            
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