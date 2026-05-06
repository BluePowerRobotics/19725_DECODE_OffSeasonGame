 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;

@Config
public class Chassis {
    public enum TEAM_COLOR {
        RED, BLUE
    }

    private static final ConvexPolygon SHOOTING_AREA_LEFT = HypParams.SHOOTING_AREA_LEFT;
    private static final ConvexPolygon SHOOTING_AREA_RIGHT = HypParams.SHOOTING_AREA_RIGHT;
    private static final ConvexPolygon BOUNDING_BOX = HypParams.BoundingBox;

    private final double maxV = HypParams.maxV;
    private final double maxOmega = HypParams.maxOmega;
    private final double WanderSpeed = HypParams.WanderSpeed;

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean RunningToPose = HypParams.InitialRunningToPose;
    private final Telemetry telemetry;

    public Chassis(HardwareMap hardwareMap, TEAM_COLOR teamColor, ActionRunner actionRunner, Telemetry telemetry) {
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;

        Pose2d startPose = (teamColor == TEAM_COLOR.RED) ?
            HypParams.startPoseRed : HypParams.startPoseBlue;
        RobotPosition.RobotPositioninit(hardwareMap, startPose);
    }

    public void setMode(boolean RunningToPose){
        this.RunningToPose = RunningToPose;
    }

    public void exchangeMode(){
        RunningToPose = !RunningToPose;
    }
    public ConvexPolygon getBoundingBox(){
        return BOUNDING_BOX;
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
        double k = maxOmega / (Math.PI / 2);
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
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0), omega));
    }
    public void update(double Kx, double Ky, double Komega){
        if(!actionRunner.isBusy()){
            double vx = Kx * maxV;
            double vy = Ky * maxV;
            //原来的controller是这样实现的，但如果各方向maxV相同应当是下面这个写法
            //double vx = Kx / Math.hypot(Kx, Ky) * maxV;
            //double vy = Ky / Math.hypot(Kx, Ky)* maxV;
            double omega = Komega * maxOmega;
            if(RunningToPose){
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vx,vy), omega));
            }
            else{
                Point2D RelVel=Point2D.rotate(new Point2D(vx, vy),-RobotPosition.getInstance().getTheta());
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(RelVel.getX(),RelVel.getY()),
                        omega));
            }
        }
    }
        public void telemetry(){
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("Use No Head Mode",useNoHeadMode);
        telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());

    }
}