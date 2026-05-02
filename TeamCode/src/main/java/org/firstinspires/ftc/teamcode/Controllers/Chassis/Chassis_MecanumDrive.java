package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

//TODO: 所以，如果你的chassis只是再套了一层封装，里面还是MecanumDrive，那为何要过度包装？
@Config
public class Chassis_MecanumDrive {
    public static Pose2d ShootPoseRED = new Pose2d(-60, -12, 0);
    public static Pose2d ShootPoseBlue = new Pose2d(-60, 12, 0);
    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean RunningToPose = false;
    double WanderSpeed;
    public Chassis_MecanumDrive(MecanumDrive drive, double WanderSpeed, ActionRunner actionRunner) {
        this.drive = drive;
        this.WanderSpeed = WanderSpeed;
        this.actionRunner = actionRunner;
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
        //todo 如果一定要套壳，你就在chassis里面维护这个
        //示范，只实现红方
        if(!RunningToPose){
            TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(ShootPoseRED.position, ShootPoseRED.heading);
            actionRunner.clear();
            actionRunner.add(actionBuilder.build());
            RunningToPose = true;
        }
        actionRunner.update();
        if(!actionRunner.isBusy()){
            RunningToPose = false;
        }
        //todo: 实现Chassis移动到射击区域
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
