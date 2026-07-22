package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.Point2D;

/**
 * 前往射击区域Action：使用ConvexPolygon计算离车最近的发射点，通过RoadRunner轨迹前往
 */
public class GoToShootingAreaAction implements Action {
    private final Action trajectoryAction;

    public GoToShootingAreaAction(MecanumDrive drive) {
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        Point2D robotPos = new Point2D(currentPose.position.x, currentPose.position.y);

        // 计算到两个射击区域边界最近点的向量
        Point2D vecLeft = HypParams.SHOOTING_AREA_LEFT.NearestVectorFrom(robotPos);
        Point2D vecRight = HypParams.SHOOTING_AREA_RIGHT.NearestVectorFrom(robotPos);

        double distLeft = vecLeft.getDistance();
        double distRight = vecRight.getDistance();

        // 选择更近的射击区域
        Point2D nearestVec = distLeft <= distRight ? vecLeft : vecRight;

        // 目标位置 = 当前位置 + 最近向量
        double targetX = robotPos.getX() + nearestVec.getX();
        double targetY = robotPos.getY() + nearestVec.getY();

        this.trajectoryAction = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(targetX, targetY), currentPose.heading.toDouble())
                .build();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // 获取当前机器人位姿，将BoundingBox转换到绝对坐标
        Pose2d currentPose = RobotPosition.getInstance().getPose2d();
        ConvexPolygon absBoundingBox = HypParams.BoundingBox.inAbsolute(currentPose);

        // 判定BoundingBox是否与任一射击区域相交，相交则立即停止
        if (absBoundingBox.IsIntersected(HypParams.SHOOTING_AREA_LEFT)
                || absBoundingBox.IsIntersected(HypParams.SHOOTING_AREA_RIGHT)) {
            packet.put("GoToShootingArea", "Arrived (intersected)");
            return false;
        }

        packet.put("GoToShootingArea", "Navigating to shooting area...");
        return trajectoryAction.run(packet);
    }
}