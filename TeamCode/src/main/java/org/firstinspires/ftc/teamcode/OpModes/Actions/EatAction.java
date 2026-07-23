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
 * 吃球Action：启动intake，沿指定方向移动distance，等待eatSeconds秒，停止intake，返回原位
 */
public class EatAction implements Action {
    private final MecanumDrive drive;
    private final Sweeper sweeper;
    private final double distance; // 移动距离（英寸，正值）
    private final double direction; // 移动方向（弧度，场心地坐标系）
    private final long eatDurationMs;
    private Action currentTrajectory;
    private int phase = 0; // 0: 前进, 1: 等待吃球, 2: 返回
    private long eatStartTime;

    /**
     * @param drive MecanumDrive实例
     * @param sweeper 吸球器
     * @param distance 吃球移动距离（英寸，正值）
     * @param direction 吃球方向（弧度，场心地坐标系，0=+X, PI/2=+Y, PI=-X, -PI/2=-Y）
     * @param eatSeconds 吃球等待时间（秒）
     */
    public EatAction(MecanumDrive drive, Sweeper sweeper, double distance, double direction, double eatSeconds) {
        this.drive = drive;
        this.sweeper = sweeper;
        this.distance = distance;
        this.direction = direction;
        this.eatDurationMs = (long) (eatSeconds * 1000);

        double dx = distance * Math.cos(direction);
        double dy = distance * Math.sin(direction);

        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        Vector2d targetPos = new Vector2d(
                currentPose.position.x + dx,
                currentPose.position.y + dy
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
                // 构建返回轨迹：沿反方向移回
                sweeper.setStop();
                Pose2d currentPose = RobotPosition.getInstance().getPose2d();
                double returnDx = distance * Math.cos(direction + Math.PI);
                double returnDy = distance * Math.sin(direction + Math.PI);
                Vector2d returnPos = new Vector2d(
                        currentPose.position.x + returnDx,
                        currentPose.position.y + returnDy
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