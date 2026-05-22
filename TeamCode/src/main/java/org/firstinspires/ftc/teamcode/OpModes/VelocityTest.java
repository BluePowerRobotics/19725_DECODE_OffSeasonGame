package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;

@TeleOp(name = "VelocityTest", group = "Tests")
public class VelocityTest extends LinearOpMode {
    private Chassis chassis;
    
    // 状态枚举
    private enum State {
        STOP, RUNNING
    }
    
    private State currentState = State.STOP;
    
    // 速度参数
    private double v = 0.0;        // 前进速度系数 (-1.0 到 1.0)
    private double omega = 0.0;    // 角速度系数 (-1.0 到 1.0)
    
    // 步长设置
    private static final double V_STEP = 0.1;        // 速度步长
    private static final double OMEGA_STEP = 0.1;    // 角速度步长
    private static final double V_STEP_HALF = 0.05;  // 扣扳机时速度步长
    private static final double OMEGA_STEP_HALF = 0.05;  // 扣扳机时角速度步长

    @Override
    public void runOpMode() {
        // 初始化底盘
        ActionRunner actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, OffseasonDECODE.TEAM_COLOR.RED, actionRunner, telemetry);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("State", currentState);
        telemetry.addData("Kv", "%.2f", v);
        telemetry.addData("Komega", "%.2f", omega);
        telemetry.addData("Instructions", "A: start | B: stop | D-pad: adjust Kv/ Komega");
        telemetry.addData("Instructions", "Right trigger: half step");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 读取右扳机状态（决定步长）
            boolean isTriggerPressed = gamepad1.right_trigger > 0.5;
            double vStep = isTriggerPressed ? V_STEP_HALF : V_STEP;
            double omegaStep = isTriggerPressed ? OMEGA_STEP_HALF : OMEGA_STEP;
            
            // A键启动（边沿触发）
            if (gamepad1.aWasPressed()) {
                currentState = State.RUNNING;
            }
            
            // B键停止（边沿触发）
            if (gamepad1.bWasPressed()) {
                currentState = State.STOP;
                chassis.stop();
            }
            
            // 方向键上下调节前进速度 v
            if (gamepad1.dpadUpWasPressed()) {
                v += vStep;
                v = Math.min(1.0, v);  // 限制最大值
            }
            if (gamepad1.dpadDownWasPressed()) {
                v -= vStep;
                v = Math.max(-1.0, v);  // 限制最小值
            }
            
            // 方向键左右调节角速度 omega
            if (gamepad1.dpadRightWasPressed()) {
                omega += omegaStep;
                omega = Math.min(1.0, omega);  // 限制最大值
            }
            if (gamepad1.dpadLeftWasPressed()) {
                omega -= omegaStep;
                omega = Math.max(-1.0, omega);  // 限制最小值
            }
            
            // 根据状态更新底盘
            if (currentState == State.RUNNING) {
                // 不做左右平移（Ky = 0）
                chassis.update(v, 0.0, omega);
            }
            
            // 实时显示当前参数
            telemetry.addData("State", currentState);
            telemetry.addData("Kv (speed)", "%.2f", v);
            telemetry.addData("Komega (angular)", "%.2f", omega);
            telemetry.addData("Trigger", isTriggerPressed ? "Half Step" : "Full Step");
            telemetry.addData("Instructions", "A: start | B: stop | D-pad: adjust");
            chassis.telemetry();
            telemetry.update();
        }
        
        // 停止底盘
        chassis.stop();
    }
}