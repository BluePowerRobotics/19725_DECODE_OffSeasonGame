package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.InstanceTelemetry;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@Config
@TeleOp(name = "OffseasonDECODE_Red", group = "AAA_OffseasonDECODE")
public class OffseasonDECODE_Red extends LinearOpMode {
    long lastNanoTime;

    public enum ROBOT_STATUS {
        EATING,
        OUTPUT,
        WAITING,
        SHOOTING
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;

    public enum TEAM_COLOR {
        RED,
        BLUE
    }

    TEAM_COLOR teamColor = TEAM_COLOR.RED;

    public enum SWEEPER_STATUS {
        EAT,
        GIVE_ARTIFACT,
        TRIGGER,
        OUTPUT,
        STOP,
        PREPARE
    }

    SWEEPER_STATUS sweeperStatus = SWEEPER_STATUS.STOP;

    public enum TURRET_STATUS {
        SHOOTING,
        STOP,
        IDLE
    }

    TURRET_STATUS turretStatus = TURRET_STATUS.STOP;
    public Chassis chassis;
    public Sweeper sweeper;
    public Shooter shooter;
    public Turret turret;
    public ActionRunner actionRunner;
    public TelemetryPacket packet = new TelemetryPacket();
    public boolean ReadyToShoot = false;
    public boolean shouldAim = false;
    public boolean shouldShoot = false;
    public boolean needshoot = false;
    public static double turretmode = 0;
    public int targetSpeed = 0;
    public static boolean useNoHeadMode = false;

    void Init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, TeamColor.RED, actionRunner, telemetry);
        useNoHeadMode = HypParams.InitialUseNoHeadMode;
    }

    void inputRobotStatus() {
        if (gamepad1.xWasPressed()) {
            robotStatus = ROBOT_STATUS.WAITING;
            useNoHeadMode = !useNoHeadMode;
        }

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            shouldShoot = false;
            shouldAim = false;
            robotStatus = ROBOT_STATUS.WAITING;
        }

        if (gamepad2.leftBumperWasPressed()) {
            shouldShoot = false;
            if (robotStatus == ROBOT_STATUS.EATING) {
                robotStatus = ROBOT_STATUS.WAITING;
            } else {
                robotStatus = ROBOT_STATUS.EATING;
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            shouldShoot = false;
            needshoot = false;
            robotStatus = ROBOT_STATUS.EATING;
        }

        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            shouldShoot = false;
            needshoot = false;
            robotStatus = ROBOT_STATUS.OUTPUT;
        }

        if ((gamepad1.yWasPressed() || gamepad2.yWasPressed()) && (turretmode != 2)) {
            shouldShoot = !shouldShoot;
            shouldAim = !shouldAim;
            needshoot = false;
            robotStatus = ROBOT_STATUS.SHOOTING;
        }

        if (gamepad2.xWasPressed()) {
            turretmode = (turretmode + 1) % 3;
        }
    }

    void setStatus() {
        switch (robotStatus) {
            case EATING:
                sweeperStatus = SWEEPER_STATUS.EAT;
                turretStatus = TURRET_STATUS.IDLE;
                break;
            case OUTPUT:
                sweeperStatus = SWEEPER_STATUS.OUTPUT;
                turretStatus = TURRET_STATUS.IDLE;
                break;
            case WAITING:
                sweeperStatus = SWEEPER_STATUS.STOP;
                turretStatus = TURRET_STATUS.IDLE;
                break;
            case SHOOTING:
                turretStatus = TURRET_STATUS.SHOOTING;
                break;
        }
    }

    void chassis() {
        chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, useNoHeadMode);
    }

    void sweeper() {
        switch (sweeperStatus) {
            case EAT:
                sweeper.setEat();
                break;
            case GIVE_ARTIFACT:
                sweeper.setGiveArtifact();
                break;
            case TRIGGER:
                sweeper.setTrigger();
                break;
            case OUTPUT:
                sweeper.setOutput();
                break;
            case STOP:
                sweeper.setStop();
                break;
            case PREPARE:
                break;
        }
        sweeper.update();
    }

    void turret() {
        switch (turretStatus) {
            case SHOOTING:
                if (turretmode == 0) {
                    turret.update(true, shouldShoot, targetTagId);
                } else if (turretmode == 1) {
                    turret.update(false, shouldShoot, targetTagId);
                } else if (turretmode == 2) {
                    shouldAim = false;
                    shouldShoot = false;
                    needshoot = false;
                    if (gamepad2.dpad_up) {
                        targetSpeed = 850;
                    } else if (gamepad2.dpad_right) {
                        targetSpeed = 760;
                    } else if (gamepad2.dpad_down) {
                        targetSpeed = 650;
                    } else if (gamepad2.dpad_left) {
                        targetSpeed = 500;
                    } else {
                        targetSpeed = (int) gamepad2.left_trigger * 1000;
                    }
                    if (gamepad2.yWasPressed()) {
                        needshoot = true;
                    }
                    turret.update(gamepad2.left_stick_x);
                    turret.update(targetSpeed, needshoot);
                }
                if (turretmode != 2) {
                    if (turret.getShootPhase() == Turret.ShootPhase.PREPARING) {
                        if (!sweeper.isPreparing()) {
                            sweeper.prepare(HypParams.PrepareAngle);
                        }
                        sweeperStatus = SWEEPER_STATUS.PREPARE;
                    } else if (turret.getShootPhase() == Turret.ShootPhase.FIRING) {
                        sweeperStatus = SWEEPER_STATUS.TRIGGER;
                    }
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

    void telemetry() {
        sweeper.setTelemetry();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.addData("READYTOSHOOT", ReadyToShoot);
        telemetry.addData("isBusy", actionRunner.isBusy());
        telemetry.addData("TeamColor", teamColor);
        telemetry.addData("RobotSTATUS", robotStatus.toString());
        telemetry.addData("turretSTATUS", turretStatus.toString());
        telemetry.addData("sweeperSTATUS", sweeperStatus.toString());
        telemetry.addData("NoHeadMode", useNoHeadMode);
        telemetry.addData("Position", RobotPosition.getInstance().getPose2d().toString());
        telemetry.addData("Heading", RobotPosition.getInstance().getPose2d());
        shooter.setTelemetry();
        chassis.telemetry();
        sweeper.setTelemetry();
        telemetry.addData("FPS", 1000000000.0 / (System.nanoTime() - lastNanoTime));
        lastNanoTime = System.nanoTime();
        telemetry.update();
    }

    public static int targetTagId = 24;
    public static double time = 1.2;
    boolean InitStarted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        while (opModeInInit() || !InitStarted) {
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