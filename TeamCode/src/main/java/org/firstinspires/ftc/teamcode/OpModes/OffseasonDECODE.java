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
import org.firstinspires.ftc.teamcode.Controllers.Turret.Turret;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@Config
@TeleOp(name = "OffseasonDECODE", group = "AAA_OffseasonDECODE")
public class OffseasonDECODE extends LinearOpMode {
    long lastNanoTime;

    // 状态机：定义机器人当前正在执行的操作
    public enum ROBOT_STATUS {
        EATING,   // 吸取器正在吃球
        OUTPUT,   // 吸取器正在吐球
        WAITING,  // 空闲待命
        SHOOTING  // 发射序列中
    }

    ROBOT_STATUS robotStatus = ROBOT_STATUS.WAITING;

    // 队伍颜色选择
    public enum TEAM_COLOR {
        RED,
        BLUE
    }

    TEAM_COLOR teamColor;

    // 炮塔瞄准模式：控制 turret.update() 使用哪种瞄准策略
    public enum TURRET_STATUS {
        VISION,        // 使用视觉(AprilTag) + 定位联合瞄准
        LOCALIZATION,  // 仅使用定位(无视觉)自动瞄准
        MANUAL         // 手柄手动控制 roll/yaw/speed
    }

    TURRET_STATUS turretStatus = TURRET_STATUS.VISION;

    private boolean initStarted = false;
    // 高层控制器：通过各控制器的 update() 接口驱动硬件，不直接访问下层类
    private Chassis chassis;
    private Sweeper sweeper;
    private Turret turret;
    private ActionRunner actionRunner;

    // 炮塔手动模式参数
    private int targetTagId;   // 自动瞄准的目标 AprilTag ID
    private double roll = 0.0; // Roll 角度，由左摇杆 X 累积
    private double yaw = 50.0; // Yaw 角度，由 D-Pad 上下调节
    private int targetSpeed = 0; // 射手转速，由 D-Pad 左右调节
    private boolean isShooting = false; // 发射开关，由 A 键切换

    // 手动模式参数调节步长和限幅
    private static final double ROLL_SPEED = 2.0;
    private static final double YAW_STEP = 5.0;
    private static final int SPEED_STEP = 100;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 3000;

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化仪表盘和全局 Telemetry 单例
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry = InstanceTelemetry.init(telemetry);

        // === Init 阶段：选择队伍颜色 ===
        while (opModeInInit() || !initStarted) {
            initStarted = true;
            if (gamepad1.a) teamColor = TEAM_COLOR.BLUE;
            if (gamepad1.b) teamColor = TEAM_COLOR.RED;

            // 根据队伍颜色设置自动瞄准目标 Tag
            switch (teamColor) {
                case BLUE: targetTagId = 20; break;
                case RED: targetTagId = 24; break;
            }

            telemetry.addData("TEAM_COLOR", teamColor != null ? teamColor.toString() : "N/A");
            telemetry.addData("Target Tag ID", targetTagId);
            telemetry.addData("Instructions", "A: Blue, B: Red");
            telemetry.update();
        }

        // === 硬件控制器初始化 ===
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry);
        sweeper = new Sweeper(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.setGamepads(gamepad1, gamepad2);

        // 操作说明
        telemetry.addData("Status", "Initialized");
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output");
        telemetry.addData("Y", "Sweeper Stop");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.addData("Left Stick X", "Turret Roll (MANUAL mode)");
        telemetry.addData("D-Pad Up/Down", "Yaw (MANUAL mode)");
        telemetry.addData("D-Pad Left/Right", "Speed (MANUAL mode)");
        telemetry.addData("A", "Toggle Shoot");
        telemetry.addData("X", "Cycle Aim Mode (VISION->LOCALIZATION->MANUAL)");
        telemetry.addData("Left Bumper", "Sweeper Eat (Priority)");
        telemetry.addData("Right Bumper", "Sweeper Output (Priority)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === 主循环：各控制器 update() 统一调度 ===
        while (opModeIsActive()) {
            // 更新定位信息
            RobotPosition.getInstance().update();

            // === 底盘控制：左摇杆驱动，X 切换无头模式 ===
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.xWasPressed()) {
                chassis.exchangeUseNoHeadMode();
            }

            // === 吸取器控制：发射状态优先于手柄 ===
            if (isShooting) {
                sweeper.setGiveArtifact();
                robotStatus = ROBOT_STATUS.SHOOTING;
            } else {
                boolean g2SweeperActive = false;
                if (gamepad2.left_bumper) {
                    sweeper.setEat();
                    robotStatus = ROBOT_STATUS.EATING;
                    g2SweeperActive = true;
                } else if (gamepad2.right_bumper) {
                    sweeper.setOutput();
                    robotStatus = ROBOT_STATUS.OUTPUT;
                    g2SweeperActive = true;
                }

                if (!g2SweeperActive) {
                    if (gamepad1.left_bumper) {
                        sweeper.setEat();
                        robotStatus = ROBOT_STATUS.EATING;
                    } else if (gamepad1.right_bumper) {
                        sweeper.setOutput();
                        robotStatus = ROBOT_STATUS.OUTPUT;
                    } else if (gamepad1.y) {
                        sweeper.setStop();
                        robotStatus = ROBOT_STATUS.WAITING;
                    } else {
                        robotStatus = ROBOT_STATUS.WAITING;
                    }
                }
            }

            // === 炮塔瞄准模式轮换：P2 X 键循环 ===
            if (gamepad2.xWasPressed()) {
                switch (turretStatus) {
                    case VISION: turretStatus = TURRET_STATUS.LOCALIZATION; break;
                    case LOCALIZATION: turretStatus = TURRET_STATUS.MANUAL; break;
                    case MANUAL: turretStatus = TURRET_STATUS.VISION; break;
                }
            }

            // === 炮塔控制：根据当前瞄准模式选择 turret.update() 接口 ===
            switch (turretStatus) {
                case VISION:
                    // 视觉+定位自动瞄准：turret 内部通过 AprilTag 和定位数据进行 Aim/Shoot
                    if (gamepad2.aWasPressed()) isShooting = !isShooting;
                    turret.update(true, isShooting, targetTagId);
                    break;
                case LOCALIZATION:
                    // 纯定位自动瞄准：turret 仅依靠场地定位数据(无视觉)
                    if (gamepad2.aWasPressed()) isShooting = !isShooting;
                    turret.update(false, isShooting, targetTagId);
                    break;
                case MANUAL:
                    // 手动模式：摇杆和 D-Pad 直接调节 Roll/Yaw/Speed
                    roll += gamepad2.left_stick_x * ROLL_SPEED;

                    if (gamepad2.dpad_up) yaw += YAW_STEP;
                    if (gamepad2.dpad_down) yaw -= YAW_STEP;

                    if (gamepad2.dpad_right) targetSpeed += SPEED_STEP;
                    if (gamepad2.dpad_left) targetSpeed -= SPEED_STEP;
                    targetSpeed = Math.max(SPEED_MIN, Math.min(SPEED_MAX, targetSpeed));

                    if (gamepad2.aWasPressed()) isShooting = !isShooting;
                    turret.update(roll, yaw, targetSpeed, isShooting);
                    break;
            }

            sweeper.update();

            // === 遥测输出 ===
            chassis.telemetry();
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());
            telemetry.addData("Robot Status", robotStatus.toString());
            telemetry.addData("Turret Aim Mode", turretStatus.toString());
            telemetry.addData("Target Tag ID", targetTagId);
            telemetry.addData("TEAM_COLOR", teamColor.toString());
            // 手动模式下额外显示角度和速度参数
            if (turretStatus == TURRET_STATUS.MANUAL) {
                telemetry.addData("Roll", "%.2f deg", roll);
                telemetry.addData("Yaw", "%.2f deg", yaw);
                telemetry.addData("Target Speed", "%d RPM", targetSpeed);
            }
            telemetry.addData("Shooting", isShooting ? "ACTIVE" : "IDLE");
            sweeper.setTelemetry();
            telemetry.addData("FPS", 1000000000.0 / (System.nanoTime() - lastNanoTime));
            lastNanoTime = System.nanoTime();
            telemetry.update();

            // Dashboard 场地俯视图：实时绘制机器人位姿
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // === 停止时关闭所有电机 ===
        chassis.stop();
        turret.stop();
        sweeper.setStop();
        sweeper.update();
    }
}