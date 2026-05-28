package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

@Config
@TeleOp(name = "OffseasonDECODE", group = "AAA_OffseasonDECODE")
public class OffseasonDECODE extends LinearOpMode {
    long lastNanoTime;
    public enum ROBOT_STATUS{
        EATING,
        OUTPUT,
        WAITING,
        SHOOTING
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;
    public enum TEAM_COLOR {
        RED,BLUE
    }
    TEAM_COLOR teamColor;

    public enum SWEEPER_STATUS {
        EAT,
        GIVE_ARTIFACT,
        OUTPUT,
        STOP
    }
    SWEEPER_STATUS sweeperStatus = SWEEPER_STATUS.STOP;
    public enum TURRET_STATUS {
        SHOOTING,
        STOP,
        IDLE
    }
    TURRET_STATUS turretStatus = TURRET_STATUS.STOP;
    public Chassis chassis; // 底盘控制器实例，负责机器人的移动控制
    public Sweeper sweeper; // 清扫器控制器实例
    public Shooter shooter; // 发射器控制器实例
    public Turret turret; // 触发器控制器实例
    public ActionRunner actionRunner;//actionRunner控制器实例
    public TelemetryPacket packet = new TelemetryPacket();
    //public BlinkinLedController ledController; // LED控制器实例
    public boolean ReadyToShoot = false;
    public boolean shouldAim = false;//建议把这个分配给二操右扳机RT//先不用扳机，两个都暂时都给
    //最好留一手：二操按x切换手动瞄准，摇杆控制炮台，防止定位瞄准失效
    public boolean shouldShoot = false;
    public boolean needshoot = false;
    public static double turretmode = 0;//0为自动瞄准，1为定位的开环，2为手动瞄准
    public int targetSpeed = 0;

    void Init(){

        //todo set team color
        teamColor = TEAM_COLOR.BLUE;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
        //ledController = new BlinkinLedController(hardwareMap);
    }
    void inputRobotStatus(){
        if(gamepad1.xWasPressed()){
            robotStatus = ROBOT_STATUS.WAITING;
            chassis.exchangeMode();
        }//为什么要用xWasReleased?

        if(gamepad1.aWasPressed() || gamepad2.aWasPressed()){
            shouldShoot = false;
            shouldAim = false;
            robotStatus = ROBOT_STATUS.WAITING;
        }

        //一操 二操切换 二操可强制开启
        if(gamepad2.leftBumperWasPressed()){
            shouldShoot = false;
            if(robotStatus == ROBOT_STATUS.EATING){
                robotStatus = ROBOT_STATUS.WAITING;
            }
            else{
                robotStatus = ROBOT_STATUS.EATING;
            }

        }

        //吸球部分
        if(gamepad1.leftBumperWasPressed()){
            shouldShoot = false;
            robotStatus = ROBOT_STATUS.EATING;
        }

        if(gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()){
            shouldShoot = false;
            robotStatus = ROBOT_STATUS.OUTPUT;
        }

        //发射部分
        if(gamepad1.yWasPressed() || gamepad2.yWasPressed()){
            shouldShoot = !shouldShoot;
            shouldAim = !shouldAim;
            robotStatus = ROBOT_STATUS.SHOOTING;
        }

//        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()){
//            shouldShoot = false;
//            robotStatus = ROBOT_STATUS.GIVE_ARTIFACT;
//        }

        if (gamepad2.xWasPressed()){
            turretmode = (turretmode + 1) % 3;
        }

    }
    void setStatus() {
        switch (robotStatus) {
            case EATING:
                sweeperStatus = SWEEPER_STATUS.EAT;
                turretStatus = TURRET_STATUS.IDLE;
                //ledController.setColor(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            case OUTPUT:
                sweeperStatus = SWEEPER_STATUS.OUTPUT;
                turretStatus = TURRET_STATUS.IDLE;
                break;
            case WAITING:
                //boolean AprilTagStatus = !Double.isNaN(aprilTagDetector.getPose().pose.position.x);
                sweeperStatus = SWEEPER_STATUS.STOP;
                turretStatus = TURRET_STATUS.IDLE;
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
        chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
    void sweeper(){
        switch (sweeperStatus){
            case EAT:
                sweeper.setEat();
                break;
            case GIVE_ARTIFACT:
                sweeper.setGiveArtifact();
                break;
            case OUTPUT:
                sweeper.setOutput();
                break;
            case STOP:
                sweeper.setStop();
                break;
        }
    }
    void turret() {
        switch (turretStatus) {
            case SHOOTING:
                if (turretmode == 0) {
                    turret.update(true, true, targetTagId);
                } else if (turretmode == 1) {
                    turret.update(false, true, targetTagId);
                } else if (turretmode == 2) {
                    shouldAim = false;
                    shouldShoot = false;
                    if (gamepad2.dpad_up) {
                        targetSpeed = 850; // 高速
                     } else if (gamepad2.dpad_right) {
                        targetSpeed = 760; // 中高速
                    } else if (gamepad2.dpad_down) {
                        targetSpeed = 650; // 中速
                    } else if (gamepad2.dpad_left) {
                        targetSpeed = 500; // 低速
                    } else {
                        targetSpeed = (int) gamepad2.left_trigger*1000;//扳机控制速度，按得越深越快
                    }
                    if (gamepad2.yWasPressed()){
                        needshoot = true;
                        //重置？
                    }
                    turret.update(gamepad2.left_stick_x);//旋转
                    turret.update(targetSpeed, needshoot);//发射（手动）
                    //未完待续（控制瞄准）[需检查]
                }
                break;
            case STOP:
                turret.update(false, false, targetTagId);
                break;
            case IDLE:
                if (turretmode == 0) {
                    turret.update(true, false, targetTagId);
                } else if (turretmode == 1) {
                    turret.update(false, false, targetTagId);
                } else if (turretmode == 2) {
                    turret.update(gamepad2.left_stick_x);
                }
                break;
        }
    }

    void telemetry(){
        sweeper.setTelemetry();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("READYTOSHOOT", ReadyToShoot);
//        telemetry.addData("DIS", disSensor.getDis());
        telemetry.addData("isBusy", actionRunner.isBusy());
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("RobotSTATUS", robotStatus.toString());
        telemetry.addData("turretSTATUS", turretStatus.toString());
        telemetry.addData("sweeperSTATUS", sweeperStatus.toString());
        telemetry.addData("Position",RobotPosition.getInstance().getPose2d().toString());
        telemetry.addData("Heading", RobotPosition.getInstance().getPose2d());
        shooter.setTelemetry();
        chassis.telemetry();
        sweeper.setTelemetry();
        telemetry.addData("FPS",1000000000.0/(System.nanoTime()-lastNanoTime));
        lastNanoTime=System.nanoTime();
        telemetry.update();
    }
    public static int targetTagId;
    public static double time=1.2;
    boolean InitStarted=false;
    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        //赛前初始化，选择队伍颜色，确定目标tag ID，设置LED等
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
            turret();
            sweeper();
            telemetry();
        }
    }
}
