package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

    

    @Override
    public void runOpMode() {
        // 初始化组件
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, TeamColor.RED, actionRunner, telemetry, false);
        tracker = new Tracker(hardwareMap);
        sweeper = new Sweeper(hardwareMap, telemetry);

        // 启动 tracker
        tracker.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 更新定位和追踪
            org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().update();
            tracker.update();
            actionRunner.update();

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
            telemetry.addData("WanderSpeed (LS Y)", "%.2f", chassis.getWanderSpeed());
            chassis.telemetry();
            telemetry.update();
        }

        // 停止所有组件
        chassis.stop();
        sweeper.setStop();
        tracker.stop();
    }
}