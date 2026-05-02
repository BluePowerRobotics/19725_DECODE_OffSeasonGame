 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;

@Config
public class Chassis{
    private static final ConvexPolygon SHOOTING_AREA_LEFT = RobotPosition.getInstance().getShootingAreaLeft();
    private static final ConvexPolygon SHOOTING_AREA_RIGHT = RobotPosition.getInstance().getShootingAreaRight();
    private static final ConvexPolygon BoundingBox = RobotPosition.getInstance().getBoundingBox();

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean RunningToPose = false;
    double WanderSpeed;
    
    public Chassis(MecanumDrive drive, double WanderSpeed, ActionRunner actionRunner) {
        this.drive = drive;
        this.WanderSpeed = WanderSpeed;
        this.actionRunner = actionRunner;
    }

    public ConvexPolygon getBoundingBox(){
        return BoundingBox;
    }
    public void stop(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0,0),
                0));
    }
    public void GoTo(double targetTheta){ //targetTheta表示目标与小车正前方（x轴正方向）的夹角（逆时针为正）
        //前进的同时转向
        double vx = Math.cos(targetTheta) * WanderSpeed;
        double vy = Math.sin(targetTheta) * WanderSpeed;
        double k = ChassisController.PARAMS.maxOmega / (Math.PI / 2);
        double omega = targetTheta * k;
        //这里的坐标系和正负我不确定。去TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/tuning/LocalizationTest.java里试
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(vx,vy),
                omega));

    }
    public void GoToShootingArea(){
        Point2D currentPos = new Point2D(
            RobotPosition.getInstance().getX(),
            RobotPosition.getInstance().getY()
        );

        if (SHOOTING_AREA_LEFT.Contains(currentPos) || SHOOTING_AREA_RIGHT.Contains(currentPos)) {
            stop();
            return;
        }
        //前往最近的发射区，可以改为只去小三角
        Point2D VecToLeft = SHOOTING_AREA_LEFT.NearestVectorFrom(currentPos);
        Point2D VecToRight = SHOOTING_AREA_RIGHT.NearestVectorFrom(currentPos);

        Point2D nearestVector = (VecToLeft.getDistance() < VecToRight.getDistance()) ? VecToLeft : VecToRight;
        double targetTheta = nearestVector.getRadian()-RobotPosition.getInstance().getTheta();

        GoTo(targetTheta);
    }
    public void SlowTurning(double omega){
        if(!actionRunner.isBusy()){
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0,0),
                    omega));
        }

    }
    public void update(){
        //todo: 实现Chassis手动模式更新
    }
}