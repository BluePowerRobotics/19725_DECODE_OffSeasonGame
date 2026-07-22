package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * 吃球Action：启动intake，沿Y轴移动EatDistance，等待EatSecond秒，停止intake，返回原位
 * 红色方：先向+Y移动再向-Y移回
 * 蓝色方：先向-Y移动再向+Y移回
 */
public class EatAction implements Action {
    private final MecanumDrive drive;
    private final Sweeper sweeper;
    private final double eatDistance; // 正数为+Y方向，负数为-Y方向
    private final long eatDurationMs;
    private Action currentTrajectory;
    private int phase = 0; // 0: 前进, 1: 等待吃球, 2: 返回
    private long eatStartTime;

    /**
     * @param drive MecanumDrive实例
     * @param sweeper 吸球器
     * @param eatDistance 吃球移动距离（英寸），正数向+Y，负数向-Y
     * @param eatSeconds 吃球等待时间（秒）
     */
    public EatAction(MecanumDrive drive, Sweeper sweeper, double eatDistance, double eatSeconds) {
        this.drive = drive;
        this.sweeper = sweeper;
        this.eatDistance = eatDistance;
        this.eatDurationMs = (long) (eatSeconds * 1000);

        // 构建前进轨迹：沿Y轴直线移动
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        Vector2d targetPos = new Vector2d(
                currentPose.position.x,
                currentPose.position.y + eatDistance
        );
        this.currentTrajectory = drive.actionBuilder(currentPose)
                .splineToConstantHeading(targetPos, currentPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        sweeper.update();
        switch (phase) {
            case 0: // 前进吃球
                sweeper.setEat();
                if (!currentTrajectory.run(packet)) {
                    phase = 1;
                    eatStartTime = System.currentTimeMillis();
                    packet.put("EatAction", "Eating...");
                }
                return true;

            case 1: // 等待吃球
                sweeper.setEat();
                if (System.currentTimeMillis() - eatStartTime < eatDurationMs) {
                    return true;
                }
                // 构建返回轨迹
                sweeper.setStop();
                Pose2d currentPose = RobotPosition.getInstance().getPose2d();
                Vector2d returnPos = new Vector2d(
                        currentPose.position.x,
                        currentPose.position.y - eatDistance
                );
                currentTrajectory = drive.actionBuilder(currentPose)
                        .splineToConstantHeading(returnPos, currentPose.heading.toDouble())
                        .build();
                phase = 2;
                packet.put("EatAction", "Returning...");
                return true;

            case 2: // 返回原位
                if (!currentTrajectory.run(packet)) {
                    packet.put("EatAction", "Done");
                    return false;
                }
                return true;

            default:
                return false;
        }
    }
}