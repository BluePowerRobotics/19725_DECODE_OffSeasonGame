package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.rubbishbin.BlinkinLedController;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@Config
@TeleOp(name = "OffseasonDECODE", group = "AAA_OffseasonDECODE")
public class OffseasonDECODE extends LinearOpMode {
    long lastNanoTime;
    public enum ROBOT_STATUS{
        EATING,
        WAITING,
        SHOOTING,
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;
    public enum TRIGGER_STATUS {
        OPEN,
        CLOSE
    }
    TRIGGER_STATUS triggerStatus = TRIGGER_STATUS.CLOSE;


    public enum SWEEPER_STATUS {
        EAT,
        GIVE_ARTIFACT,
        OUTPUT,
        STOP
    }
    SWEEPER_STATUS sweeperStatus = SWEEPER_STATUS.STOP;
    public enum SHOOTER_STATUS {
        SHOOTING,
        STOP
    }
    SHOOTER_STATUS shooterStatus = SHOOTER_STATUS.STOP;
    public Chassis chassis; // 底盘控制器实例，负责机器人的移动控制
    public Sweeper sweeper; // 清扫器控制器实例
    public Shooter shooter; // 发射器控制器实例
    public Turret turret; // 触发器控制器实例
    //public BlinkinLedController ledController; // LED控制器实例
    //    AprilTagDetector aprilTagDetector;
    public static int tmpSpeed = 700;
    //暂时关闭发射时的速度限制
    public static int OpenSweeperSpeedThreshold = 1000;
    //
//    public double targetSpeed = ShooterAction.speed35_55;
    public static double AdditionK = 0.01;
    //targetSpeed乘上Kspeed才是真实速度，修正发射速度
    double Kspeed = 1;
    boolean directControl=false;
    int currentPosition = 0;
    //修正角度偏差（从读取的底层修改）
    //todo 让degreeOffset生效 @gyw
    public double degreeOffset = 0;
    public static double startShootingHeading = Math.PI / 2;
    public boolean ReadyToShoot = false;
    public double WanderSpeed = 0.5;
    public Boolean RunningToPose = true;

    void Init(){

        //todo set team color
        teamColor = TEAM_COLOR.RED;
        Pose2d start= new Pose2d(new Vector2d(0,0),0); //根据场地和队伍颜色调整
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, Chassis.TEAM_COLOR.RED, actionRunner, telemetry);
        //ledController = new BlinkinLedController(hardwareMap);
    }
    void inputRobotStatus(){
//        if(gamepad1.dpadRightWasPressed()){
//            KtoleranceHeading += 0.5;
//        }
//        if(gamepad1.dpadLeftWasPressed()){
//            if (teamColor == TEAM_COLOR.RED) {
//                targetSpeed = SolveShootPoint.solveShootSpeed(SolveShootPoint.solveREDShootDistance(pose));
//            }
//            if (teamColor == TEAM_COLOR.BLUE) {
//                targetSpeed = SolveShootPoint.solveShootSpeed(SolveShootPoint.solveBLUEShootDistance(pose));
//            }
//        }
//
////        if(gamepad1.dpadUpWasPressed()){
////            if(teamColor == TEAM_COLOR.BLUE){
////                chassis.resetPosition(BlueResetPose);
////            }
////            if(teamColor == TEAM_COLOR.RED){
////                chassis.resetPosition(RedResetPose);
////            }
////        }
        if(gamepad1.xWasPressed()){
            robotStatus = ROBOT_STATUS.WAITING;
            chassis.exchangeMode();
        }
//        if(gamepad2.bWasPressed()){
//            targetSpeed = tmpSpeed;
//        }
//
//
//        //二操的修正功能
//        if(gamepad2.dpadLeftWasPressed()){
//            degreeOffset -= AdditionDegree;
//            chassis.robotPosition.mecanumDrive.localizer.setPose(new Pose2d(chassis.robotPosition.getData().getPose2d().position, chassis.robotPosition.getData().headingRadian + Math.toRadians(-AdditionDegree)));
//        }
//        if(gamepad2.dpadRightWasPressed()){
//            degreeOffset += AdditionDegree;
//            chassis.robotPosition.mecanumDrive.localizer.setPose(new Pose2d(chassis.robotPosition.getData().getPose2d().position, chassis.robotPosition.getData().headingRadian + Math.toRadians(AdditionDegree)));
//        }
//        if(gamepad2.dpadDownWasPressed()){
//            Kspeed -= AdditionK;
//        }
//        if(gamepad2.dpadUpWasPressed()){
//            Kspeed += AdditionK;
//        }
        if(gamepad2.yWasPressed()  ){
            robotStatus = ROBOT_STATUS.SHOOTING;
        }

        if(gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            ReadyToShoot = false;
            robotStatus = ROBOT_STATUS.WAITING;
        }

        //一操 二操切换 二操可强制开启
        if(gamepad2.leftBumperWasPressed()){
            ReadyToShoot = false;
            if(robotStatus == ROBOT_STATUS.EATING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.EATING;
            }

        }

        if(gamepad1.leftBumperWasPressed()){
            ReadyToShoot = false;
            robotStatus = ROBOT_STATUS.EATING;
        }
//        else if((gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) && robotStatus != ROBOT_STATUS.CLIMBING){
//            ReadyToShoot = false;
//            if(robotStatus == ROBOT_STATUS.OUTPUTTING){
//                robotStatus = ROBOT_STATUS.WAITING;
//            }
//            else{
//                robotStatus = ROBOT_STATUS.OUTPUTTING;
//            }
//        }

//        if(gamepad1.right_trigger > 0.6){
//            double heading = 0;
//            if (teamColor == TEAM_COLOR.RED) {
//                heading = SolveEatPoint.solveREDEatHeading(pose);
//            }
//            if (teamColor == TEAM_COLOR.BLUE) {
//                heading = SolveEatPoint.solveBLUEEatHeading(pose);
//            }
//            //todo
//            chassis.setHeadingLockRadian(heading);
//        }

//        todo: 发射部分
        if(gamepad1.yWasPressed()){
            ReadyToShoot = !ReadyToShoot;
            if(ReadyToShoot){
                double heading = 0;
                if (teamColor == TEAM_COLOR.RED) {

                }
                if (teamColor == TEAM_COLOR.BLUE) {

                }
            }
        }

//        if(ReadyToShoot){
//            double currentHeading = 0;//todo 位置角度正确值设定
//            double targetHeading = chassis.getHeadingLockRadian(); //不知道什么意思，应该会改用update
//            if(Math.abs(currentHeading - targetHeading) < startShootingHeading){
//                robotStatus = ROBOT_STATUS.SHOOTING;
//            }
//        }
    }
    void setStatus() {
        switch (robotStatus) {
            case EATING:
                sweeperStatus = SWEEPER_STATUS.EAT;
                shooterStatus = SHOOTER_STATUS.STOP;
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case WAITING:
                //boolean AprilTagStatus = !Double.isNaN(aprilTagDetector.getPose().pose.position.x);
                sweeperStatus = SWEEPER_STATUS.STOP;
                shooterStatus = SHOOTER_STATUS.STOP;
                if (teamColor == TEAM_COLOR.RED) {
////                    if(AprilTagStatus){
////                        ledController.setColor(LedPreset.HEARTBEAT_RED.getPattern());
////                    }
////                    else{
////                        ledController.showRedTeam();
////                    }
                    //ledController.showRedTeam();
                }
                else{
////                    if(AprilTagStatus){
////                        ledController.setColor(LedPreset.HEARTBEAT_BLUE.getPattern());
////                    }
////                    else{
////                        ledController.showBlueTeam();
////                    }
                    //ledController.showBlueTeam();
                }
                break;
            case SHOOTING:
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                shooterStatus = SHOOTER_STATUS.SHOOTING;
                //sweeper和trigger状态由shooter条件决定，在shoot()中
                break;


        }
    }
    public static int targetTagId;
    long lastSetTimeMS=0;
    boolean showSpeedColor=false;
    public static double time=1.2;
    public static double time_2=0.3;
    public static int IntervalMS=1;
    boolean InitStarted=false;
    @Override
    public void runOpMode() throws InterruptedException {
        //留下更改一些参数的后门(??
        while(opModeInInit()||!InitStarted) {
            if (gamepad1.a) {
                teamColor = TEAM_COLOR.BLUE;
            }
            if (gamepad1.b) {
                teamColor = TEAM_COLOR.RED;
            }
            // 确定目标tag ID
            switch (teamColor) {
                case BLUE:
                    targetTagId = 20; // 默认蓝队tag ID
                    break;
                case RED:
                    targetTagId = 24;
                    break;
            }
            if(teamColor == TEAM_COLOR.BLUE){
                //ledController.showBlueTeam();
            }
            if(teamColor == TEAM_COLOR.RED){
                //ledController.showRedTeam();
            }
//            if(teamColor == TEAM_COLOR.BLUE){
//                chassis.resetNoHeadModeStartError(-Math.PI/2);
//            }
//            else{
//                chassis.resetNoHeadModeStartError(Math.PI/2);
//            }
//            telemetry.addData("Position(inch)", Point2D.rotate(chassis.robotPosition.getData().getPosition(DistanceUnit.INCH), teamColor == TEAM_COLOR.BLUE ? Math.PI / 2 : -Math.PI / 2).toString());
            telemetry.addData("TEAM_COLOR", teamColor.toString());
            telemetry.addData("FPS", 1000000000.0 / (System.nanoTime() - lastNanoTime));
            telemetry.update();
            InitStarted = true;
        }

        waitForStart();
        while (opModeIsActive()) {
            Init();
            inputRobotStatus();
            setStatus();
        }
    }
}
