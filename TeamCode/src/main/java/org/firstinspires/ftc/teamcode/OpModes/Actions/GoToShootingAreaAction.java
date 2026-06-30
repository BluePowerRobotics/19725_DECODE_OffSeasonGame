package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class GoToShootingAreaAction implements Action {
    private final Chassis chassis;
    private final Sweeper sweeper;
    private Action trajectoryAction;
    private boolean trajectoryStarted;

    public GoToShootingAreaAction(Chassis chassis, Sweeper sweeper) {
        this.chassis = chassis;
        this.sweeper = sweeper;
        this.trajectoryStarted = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        RobotPosition.getInstance().update();

        sweeper.setStop();
        sweeper.update();

        // 如果已经到达射击区域，停止并结束
        if (RobotPosition.getInstance().isAbleToShoot()) {
            chassis.stop();
            return false;
        }

        // 如果轨迹还没开始，构造并启动轨迹
        if (!trajectoryStarted) {
            Pose2d currentPose = RobotPosition.getInstance().getPose2d();
            Point2D currentPos2D = new Point2D(currentPose.position.x, currentPose.position.y);

            // 选择射击区域并计算最近点
            ConvexPolygon shootingArea = HypParams.ToLeft
                ? HypParams.SHOOTING_AREA_LEFT
                : HypParams.SHOOTING_AREA_RIGHT;
            Point2D nearestVec = shootingArea.NearestVectorFrom(currentPos2D);
            double targetX = currentPos2D.getX() + nearestVec.getX();
            double targetY = currentPos2D.getY() + nearestVec.getY();

            trajectoryAction = RobotPosition.getInstance().getDrive().actionBuilder(currentPose)
                .turnTo(Math.PI)
                .strafeTo(new Vector2d(targetX, targetY))
                .build();

            trajectoryStarted = true;
        }

        // 执行轨迹
        if (trajectoryAction != null) {
            boolean running = trajectoryAction.run(packet);
            if (!running) {
                // 轨迹执行完毕
                trajectoryStarted = false;
                trajectoryAction = null;
            }
        }

        // 继续执行，直到到达射击区域
        return true;
    }
}