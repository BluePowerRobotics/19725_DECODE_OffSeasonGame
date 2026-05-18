package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TurretTest", group = "Tests")
public class TurretTest extends LinearOpMode {
    private Turret turret;
    
    // 手动控制参数
    private double roll = 0.0;      // 水平旋转角（度）
    private double yaw = 0.0;       // 仰角（度）
    private int speed = 3000;       // 飞轮转速
    
    // 步长设置
    private static final double YAW_STEP = 5.0;        // 仰角步长（度）
    private static final int SPEED_STEP = 50;          // 转速步长
    private static final double YAW_STEP_HALF = 2.5;   // 扣扳机时仰角步长
    private static final int SPEED_STEP_HALF = 25;     // 扣扳机时转速步长
    
    // 角度限制
    private static final double ROLL_MIN = -180.0;
    private static final double ROLL_MAX = 180.0;
    private static final double YAW_MIN = 45.0;
    private static final double YAW_MAX = 65.0;
    private static final int SPEED_MIN = 0;
    private static final int SPEED_MAX = 800;

    @Override
    public void runOpMode() {
        // 初始化炮塔
        turret = new Turret(hardwareMap, telemetry);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left stick: adjust roll | D-pad: adjust yaw/speed");
        telemetry.addData("Instructions", "Right trigger: half step | A button: shoot");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 读取右扳机状态（决定步长）
            boolean isTriggerPressed = gamepad1.right_trigger > 0.5;
            double yawStep = isTriggerPressed ? YAW_STEP_HALF : YAW_STEP;
            int speedStep = isTriggerPressed ? SPEED_STEP_HALF : SPEED_STEP;
            
            // 左摇杆左右控制roll（水平旋转角）
            double stickX = gamepad1.left_stick_x;
            if (Math.abs(stickX) > 0.1) {
                roll += stickX * 2.0;  // 摇杆灵敏度系数
                // 限制roll范围
                roll = Math.max(ROLL_MIN, Math.min(ROLL_MAX, roll));
            }
            
            // 方向键上下控制仰角（yaw）
            if (gamepad1.dpadUpWasPressed()) {
                yaw += yawStep;
                yaw = Math.min(YAW_MAX, yaw);
            }
            if (gamepad1.dpadDownWasPressed()) {
                yaw -= yawStep;
                yaw = Math.max(YAW_MIN, yaw);
            }
            
            // 方向键左右控制飞轮转速
            if (gamepad1.dpadRightWasPressed()) {
                speed += speedStep;
                speed = Math.min(SPEED_MAX, speed);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                speed -= speedStep;
                speed = Math.max(SPEED_MIN, speed);
            }
            
            // A键发射
            boolean shouldShoot = gamepad1.aWasPressed();
            
            // 更新炮塔状态
            turret.update(roll, yaw, speed, shouldShoot, 20);
            
            // 实时显示当前参数
            telemetry.addData("Status", "Running");
            telemetry.addData("Roll", "%.2f deg", roll);
            telemetry.addData("Yaw", "%.2f deg", yaw);
            telemetry.addData("Speed", "%d RPM", speed);
            telemetry.addData("Trigger", isTriggerPressed ? "Half Step" : "Full Step");
            telemetry.addData("Ready to Shoot", shouldShoot ? "Yes" : "No");
            telemetry.addData("DeltaH", "%.2f m", turret.getDeltaH());
            telemetry.update();
        }
        
        // 停止炮塔
        turret.stop();
        turret.close();
    }
}