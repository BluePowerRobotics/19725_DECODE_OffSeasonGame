package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.OpModes.Actions.EatAction;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.TeamColor;

@TeleOp
(name = "EatTest", group = "Tests")
public class EatTest extends LinearOpMode {
    private Chassis chassis;
    private Tracker tracker;
    private Sweeper sweeper;
    private ActionRunner actionRunner;

    // PID热调参变量：使用手柄方向键实时调整
    private double kP = 3.0, kI = 0.1, kD = 0.05;
    private static final double KP_STEP = 0.5;
    private static final double KI_STEP = 0.05;
    private static final double KD_STEP = 0.01;

    // 按键边沿检测，防止长按重复触发
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLeftBumper, prevRightBumper, prevY;

    @Override
    public void runOpMode() {
        // 初始化组件
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, TeamColor.RED, actionRunner, telemetry);
        tracker = new Tracker(hardwareMap);
        sweeper = new Sweeper(hardwareMap, telemetry);

        // 应用初始PID参数
        chassis.setHeadingPID(kP, kI, kD);

        // 启动 tracker
        tracker.start();

        telemetry.addData("Status", "Initialized - TeleOp PID Tuning Mode");
        telemetry.addData("Instructions", "D-pad: kP+/-, kI+/- | Bumpers: kD+/- | Y: Reset PID");
        telemetry.addData("Left Stick Y", "Adjust WanderSpeed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 更新定位和追踪
            org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().update();
            tracker.update();
            actionRunner.update();

            // ===== 手柄热调参 =====
            handlePIDTuning();

            // ===== 左摇杆调节WanderSpeed =====
            // FTC SDK: left_stick_y 上推为负，下推为正，取反使上推增加速度
            double speedInput = -gamepad1.left_stick_y;
            // 施加死区和平方曲线，提高低速控制精度
            if (Math.abs(speedInput) < 0.05) speedInput = 0;
            speedInput = Math.signum(speedInput) * speedInput * speedInput;
            // 映射到[0, 3]范围（假设maxV约为3）
            chassis.setWanderSpeed(speedInput * HypParams.maxV);

            // ===== 自动吸球逻辑 =====
            // 如果没有正在执行的动作
            if (!actionRunner.isBusy()) {
                // 检查视野内是否有球
                if (tracker.getHasTarget() && !org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isFull()) {
                    // 创建并执行 EatAction
                    EatAction eatAction = new EatAction(chassis, tracker,sweeper);
                    actionRunner.add(eatAction);
                    telemetry.addData("Action", "Started EatAction");
                } else {
                    // 没有球，停止底盘和清扫器
                    chassis.stop();
                    sweeper.setStop();
                    telemetry.addData("Status", "Waiting for target...");
                }
            } else {
                telemetry.addData("Action", "Eating...");
            }

            // 显示状态信息
            telemetry.addData("Has Target", tracker.getHasTarget());
            telemetry.addData("Target Theta", "%.2f deg", Math.toDegrees(tracker.getTargetTheta()));
            telemetry.addData("Is Busy", actionRunner.isBusy());
            telemetry.addData("Ball Full", org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isFull());
            telemetry.addData("Ball Empty", org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isEmpty());
            telemetry.addData("--- PID Tuning ---", "");
            telemetry.addData("kP (D-pad U/D)", "%.2f", kP);
            telemetry.addData("kI (D-pad L/R)", "%.4f", kI);
            telemetry.addData("kD (L/R Bumper)", "%.4f", kD);
            telemetry.addData("Reset PID", "Press Y");
            telemetry.addData("WanderSpeed (LS Y)", "%.2f", chassis.getWanderSpeed());
            chassis.telemetry();
            telemetry.update();
        }

        // 停止所有组件
        chassis.stop();
        sweeper.setStop();
        tracker.stop();
    }

    /**
     * 手柄方向键热调PID参数
     * D-pad Up/Down: 调整kP
     * D-pad Left/Right: 调整kI
     * Left/Right Bumper: 调整kD
     * Y: 重置PID积分和微分
     */
    private void handlePIDTuning() {
        boolean changed = false;

        // D-pad Up: kP+
        if (gamepad1.dpad_up && !prevDpadUp) {
            kP += KP_STEP;
            changed = true;
        }
        // D-pad Down: kP-
        if (gamepad1.dpad_down && !prevDpadDown) {
            kP = Math.max(0, kP - KP_STEP);
            changed = true;
        }
        // D-pad Left: kI+
        if (gamepad1.dpad_left && !prevDpadLeft) {
            kI += KI_STEP;
            changed = true;
        }
        // D-pad Right: kI-
        if (gamepad1.dpad_right && !prevDpadRight) {
            kI = Math.max(0, kI - KI_STEP);
            changed = true;
        }
        // Left Bumper: kD+
        if (gamepad1.left_bumper && !prevLeftBumper) {
            kD += KD_STEP;
            changed = true;
        }
        // Right Bumper: kD-
        if (gamepad1.right_bumper && !prevRightBumper) {
            kD = Math.max(0, kD - KD_STEP);
            changed = true;
        }

        if (changed) {
            chassis.setHeadingPID(kP, kI, kD);
        }

        // Y: 重置PID
        if (gamepad1.y && !prevY) {
            chassis.resetHeadingPID();
        }

        // 保存当前按键状态用于边沿检测
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevY = gamepad1.y;
    }
}