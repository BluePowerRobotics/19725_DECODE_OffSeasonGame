package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.OpModes.Actions.EatAction;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@Autonomous
(name = "EatTest", group = "Tests")
public class EatTest extends LinearOpMode {
    private Chassis chassis;
    private Tracker tracker;
    private Sweeper sweeper;
    private ActionRunner actionRunner;
    
    @Override
    public void runOpMode() {
        // 初始化组件
        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, OffseasonDECODE.TEAM_COLOR.RED, actionRunner, telemetry);
        tracker = new Tracker(hardwareMap);
        sweeper = new Sweeper(hardwareMap, telemetry);
        
        // 启动 tracker
        tracker.start();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Wait for target, then auto-eat");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 更新定位和追踪
            org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().update();
            tracker.update();
            actionRunner.update();
            
            // 如果没有正在执行的动作
            if (!actionRunner.isBusy()) {
                // 检查视野内是否有球
                if (tracker.getHasTarget() && !org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isFull()) {
                    // 创建并执行 EatAction
                    EatAction eatAction = new EatAction(chassis, tracker, sweeper);
                    actionRunner.add(eatAction);
                    telemetry.addData("Action", "Started EatAction");
                } else {
                    // 没有球，停止底盘和清扫器
                    chassis.stop();
                    sweeper.setStop();
                    sweeper.update();
                    telemetry.addData("Status", "Waiting for target...");
                }
            } else {
                telemetry.addData("Action", "Eating...");
            }
            
            // 显示状态信息
            telemetry.addData("Has Target", tracker.getHasTarget());
            telemetry.addData("Is Busy", actionRunner.isBusy());
            telemetry.addData("Ball Full", org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isFull());
            telemetry.addData("Ball Empty", org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition.getInstance().isEmpty());
            chassis.telemetry();
            telemetry.update();
        }
        
        // 停止所有组件
        chassis.stop();
        sweeper.setStop();
        tracker.stop();
    }
}