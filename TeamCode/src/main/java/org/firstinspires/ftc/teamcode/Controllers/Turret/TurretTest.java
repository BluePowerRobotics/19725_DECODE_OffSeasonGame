package org.firstinspires.ftc.teamcode.Controllers.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpModes.OffseasonDECODE;

@TeleOp(name = "Turret Test", group = "Test")
public class TurretTest extends LinearOpMode {
    public enum TEAM_COLOR {
        RED,BLUE
    }
    OffseasonDECODE.TEAM_COLOR teamColor;
    private Turret turret;
    public int targetTagId;
    boolean InitStarted = false;
    boolean shouldShoot = false;
    boolean shouldAim = true;
    
    @Override
    public void runOpMode() {
        while(opModeInInit()||!InitStarted) {
            if (gamepad1.a) {
                teamColor = OffseasonDECODE.TEAM_COLOR.BLUE;
            }
            if (gamepad1.b) {
                teamColor = OffseasonDECODE.TEAM_COLOR.RED;
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
        }
        // 初始化 Turret
        turret = new Turret(hardwareMap, telemetry);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Delta H", turret.getDeltaH());
        telemetry.addData("k", 100);
        telemetry.addData("b", 50);
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // 检查是否按下 A 键
            shouldShoot = gamepad1.aWasPressed();
            shouldShoot = gamepad1.bWasPressed();

            
            // 每帧调用 update 方法
            turret.update(shouldAim,shouldShoot,targetTagId);
            
            // 显示信息
            telemetry.addData("Status", "Running");
            telemetry.addData("Should Shoot", shouldShoot);
            telemetry.update();
        }
        
        // 停止时关闭视觉门户
        turret.close();
    }
}
