package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.Tracker;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.RobotPosition;
import org.firstinspires.ftc.teamcode.utility.MathSolver;


@Autonomous
(name = "KeepTrackTest", group = "Tests")
public class KeepTrackTest extends LinearOpMode {
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
            RobotPosition.getInstance().update();
            
            if (RobotPosition.getInstance().isFull()) {
                sweeper.setStop();
            }
            else{
                sweeper.setEat();
            }

            sweeper.update();
            tracker.update();
            
            if (tracker.getHasTarget()) {
                double targetTheta = tracker.getTargetTheta();
                chassis.HeadTo(MathSolver.normalizeAngle(RobotPosition.getInstance().getTheta()+targetTheta));
            } else {
                chassis.stop();
            }

            
            // 显示状态信息
            telemetry.addData("Has Target", tracker.getHasTarget());
            telemetry.addData("TargetTheta", tracker.getTargetTheta());
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