package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.rubbishbin.BlinkinLedController;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.Point2D;

@Config
@TeleOp(name = "OffseasonDECODE", group = "AAA_OffseasonDECODE")
public class OffseasonDECODE extends LinearOpMode {
    long lastNanoTime;
    public enum ROBOT_STATUS{
        EATING,
        WAITING,
        SHOOTING
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;
//    public enum TRIGGER_STATUS {
//        OPEN,
//        CLOSE
//    }
//    TRIGGER_STATUS triggerStatus = TRIGGER_STATUS.CLOSE;


    public enum SWEEPER_STATUS {
        EAT,
        GIVE_ARTIFACT,
        OUTPUT,
        STOP
    }
    SWEEPER_STATUS sweeperStatus = SWEEPER_STATUS.STOP;
    public enum TURRET_STATUS {
        SHOOTING,
        STOP
    }
    TURRET_STATUS turretStatus = TURRET_STATUS.STOP;
    public Chassis chassis; // 底盘控制器实例，负责机器人的移动控制
    public Sweeper sweeper; // 清扫器控制器实例
    public Shooter shooter; // 发射器控制器实例
    public Turret turret; // 触发器控制器实例
    public ActionRunner actionRunner;//actionRunner控制器实例
    public RobotPosition robotPosition;//定位控制器实例
    //public BlinkinLedController ledController; // LED控制器实例
    public boolean ReadyToShoot = false;
    public boolean shouldAim = false;
    public boolean shouldShoot = false;

    void Init(){

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
        //ledController = new BlinkinLedController(hardwareMap);
    }
    void inputRobotStatus(){
        if(gamepad1.xWasPressed()){
            robotStatus = ROBOT_STATUS.WAITING;
            chassis.exchangeMode();
        }

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

//        todo: 发射部分
        if(gamepad1.yWasPressed()){
            shouldShoot = !shouldShoot;
        }

    }
    void setStatus() {
        switch (robotStatus) {
            case EATING:
                sweeperStatus = SWEEPER_STATUS.EAT;
                turretStatus = TURRET_STATUS.STOP;
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case WAITING:
                //boolean AprilTagStatus = !Double.isNaN(aprilTagDetector.getPose().pose.position.x);
                sweeperStatus = SWEEPER_STATUS.STOP;
                turretStatus = TURRET_STATUS.STOP;
//                if (teamColor == TEAM_COLOR.RED) {
//                    //ledController.showRedTeam();
//                }
//                else{
//                    //ledController.showBlueTeam();
//                }
                break;
            case SHOOTING:
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                turretStatus = TURRET_STATUS.SHOOTING;
                //sweeper和trigger状态由shooter条件决定，在shoot()中
                break;


        }
    }
    void chassis(){
        chassis.update(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        if(gamepad1.xWasReleased())chassis.exchangeMode();
        lastNanoTime=System.nanoTime();
        chassis.telemetry();
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    void sweeper(){
        switch (sweeperStatus){
            case EAT:
                sweeper.setEat();
            case GIVE_ARTIFACT:
                sweeper.setGiveArtifact();
            case OUTPUT:
                sweeper.setOutput();
            case STOP:
                sweeper.setStop();
        }
    }
//    void turret(){
//    }
    void update(){
        sweeper.update();
        telemetry.update();
        // 更新炮塔状态
        turret.update(shouldAim,shouldShoot, targetTagId);
    }
    void telemetry(){
        sweeper.setTelemetry();
        telemetry.addData("READYTOSHOOT", ReadyToShoot);
//        telemetry.addData("DIS", disSensor.getDis());
//        telemetry.addData("RealTargetSpeed", targetSpeed * Kspeed);
//        telemetry.addData("DegreeOffset", degreeOffset);
//        telemetry.addData("NoHeadMode",chassis.getUseNoHeadMode()?"PlayerBased":"RoboticBased");
        telemetry.addData("isBusy", actionRunner.isBusy());
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("RobotSTATUS", robotStatus.toString());
//        telemetry.addData("NoHeadModeStartError:",chassis.noHeadModeStartError);
//        telemetry.addData("RunMode",chassis.runningToPoint?"RUNNING_TO_POINT":"MANUAL");
//        telemetry.addData("shooterSTATUS", shooterStatus.toString());
//        telemetry.addData("sweeperSTATUS", sweeperStatus.toString());
//        telemetry.addData("triggerSTATUS", triggerStatus.toString());
//        telemetry.addData("Position(mm)",chassis.robotPosition.getData().getPosition(DistanceUnit.MM).toString());
        telemetry.addData("SweeperSpeeed", sweeper.getPower());//或许是getVel()
        telemetry.addData("SweeperPower * 1000", sweeper.getPower() * 1000);
        telemetry.addData("Heading", robotPosition.getPose2d());
//        telemetry.addData("targetHeading",chassis.getHeadingLockRadian());
//        telemetry.addData("Position(inch)", Point2D.rotate(robotPosition.getData().getPosition(DistanceUnit.INCH),teamColor==TEAM_COLOR.BLUE?Math.PI/2:-Math.PI/2).toString());
        telemetry.addData("1-power * 1000", shooter.getPowerL() * 1000);
        telemetry.addData("2-power * 1000", shooter.getPowerR() * 1000);
        telemetry.addData("1-speed", shooter.getSpeedL());
        telemetry.addData("2-speed", shooter.getSpeedR());
        telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
        telemetry.update();
        lastNanoTime=System.nanoTime();
    }
    public static int targetTagId;
    public static double time=1.2;
    boolean InitStarted=false;
    @Override
    public void runOpMode() throws InterruptedException {
        Init();
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
//            if(teamColor == TEAM_COLOR.BLUE){
//                //ledController.showBlueTeam();
//            }
//            if(teamColor == TEAM_COLOR.RED){
//                //ledController.showRedTeam();
//            }
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
            inputRobotStatus();
            setStatus();
            chassis();
            sweeper();
            update();
            telemetry();
        }
    }
}
