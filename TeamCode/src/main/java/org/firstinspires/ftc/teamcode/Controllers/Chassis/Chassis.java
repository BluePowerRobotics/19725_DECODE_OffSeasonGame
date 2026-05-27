 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.OffseasonDECODE;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.MathSolver;



@Config
public class Chassis {
    private static final ConvexPolygon SHOOTING_AREA_LEFT = HypParams.SHOOTING_AREA_LEFT;
    private static final ConvexPolygon SHOOTING_AREA_RIGHT = HypParams.SHOOTING_AREA_RIGHT;

    private final double maxV = HypParams.maxV;
    private final double maxOmega = HypParams.maxOmega;
    private final double WanderSpeed = HypParams.WanderSpeed;

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean RunningToPose = HypParams.InitialRunningToPose;
    private boolean useFieldRelative = false;
    private final Telemetry telemetry;
    private double lastKx = 0, lastKy = 0, lastKomega = 0;

    public Chassis(HardwareMap hardwareMap, OffseasonDECODE.TEAM_COLOR teamColor, ActionRunner actionRunner, Telemetry telemetry) {
        Pose2d startPose = (teamColor == OffseasonDECODE.TEAM_COLOR.RED) ?
                HypParams.startPoseRed : HypParams.startPoseBlue;
        RobotPosition.RobotPositioninit(hardwareMap, startPose);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;

    }

    public void setMode(boolean RunningToPose){
        this.RunningToPose = RunningToPose;
    }

    public void exchangeMode(){
        RunningToPose = !RunningToPose;
    }
    public void exchangeFieldRelativeMode(){
        useFieldRelative = !useFieldRelative;
    }
    public boolean getFieldRelativeMode(){
        return useFieldRelative;
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
        double k = maxOmega / HypParams.MaxBearing;
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

        if (RobotPosition.getInstance().isAbleToShoot()) {
            stop();
            return;
        }
        //前往最近的发射区，可以改为只去小三角
        Point2D VecToLeft = SHOOTING_AREA_LEFT.NearestVectorFrom(currentPos);
        Point2D VecToRight = SHOOTING_AREA_RIGHT.NearestVectorFrom(currentPos);

        Point2D nearestVector = (HypParams.ToLeft) ? VecToLeft : VecToRight;
        double targetTheta = nearestVector.getRadian()-RobotPosition.getInstance().getTheta();

        GoTo(targetTheta);
    }

    public void HeadTo(double Theta){
        double diff = Theta - RobotPosition.getInstance().getTheta();
        double targetTheta = Math.atan2(Math.sin(diff), Math.cos(diff));
        double k = maxOmega / Math.PI;
        double omega = Math.max(-maxOmega, Math.min(maxOmega, targetTheta * k));
        drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(0,0),
            omega));
    }
    public void update(double Kx, double Ky, double Komega){
        lastKx = Kx;
        lastKy = Ky;
        lastKomega = Komega;
        if(!actionRunner.isBusy()){
            double vx = Kx * maxV;
            double vy = Ky * maxV;
            double omega = Komega * maxOmega;
            if(useFieldRelative){
                double theta = RobotPosition.getInstance().getTheta();
                double vxField = vx * Math.cos(theta) - vy * Math.sin(theta);
                double vyField = vx * Math.sin(theta) + vy * Math.cos(theta);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vyField, -vxField), omega));
            }
            else{
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vy, -vx), omega));
            }
        }
    }
    public void telemetry(){
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("RunningToPose", RunningToPose);
        telemetry.addData("FieldRelative", useFieldRelative);
        telemetry.addData("lfP",drive.leftFront.getPower());
        telemetry.addData("rfP",drive.rightFront.getPower());
        telemetry.addData("lbP",drive.leftBack.getPower());
        telemetry.addData("rbP",drive.rightBack.getPower());

        //telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        //telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        //telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());
        telemetry.addData("LeftStickX", lastKx);
        telemetry.addData("LeftStickY", lastKy);
        telemetry.addData("RightStickX", lastKomega);

    }
}