package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ChassisLocalization_Encoder {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Telemetry telemetry;

    private ElapsedTime runtime = new ElapsedTime();

    // 机械参数（定位相关）
    public static double L = 15.0;                 // 底盘前后轮中心纵向距离，单位：cm
    public static double W = 15.0;                 // 底盘左右轮中心横向距离，单位：cm
    public static double D = Math.sqrt(L*L+W*W);    // 复合参数，无需修改

    public static double EncodertoCm = 0.0255533;  // 编码器脉冲 -> cm 的工程参数

    // 编码器历史值
    private int lastFL_enc = 0, lastFR_enc = 0, lastBL_enc = 0, lastBR_enc = 0;

    // 定位参数
    private double lastTime = 0.0;
    public double x = 0.0, y = 0.0, theta = 0.0;

    public ChassisLocalization_Encoder(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br, Telemetry telemetry) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
        this.telemetry = telemetry;
    }

    // 初始化（重置计时与读取初始编码器/位姿）
    public void init(){
        runtime.reset();
        lastTime = runtime.seconds();
        lastFL_enc = frontLeft.getCurrentPosition();
        lastFR_enc = frontRight.getCurrentPosition();
        lastBL_enc = backLeft.getCurrentPosition();
        lastBR_enc = backRight.getCurrentPosition();
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }

    // 主定位函数（调用频率不宜过高）
    public void Localization() {
        double currentTime = runtime.seconds();
        double dt = currentTime - lastTime;
        if (dt < 0.01) return;

        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        int deltaFL = currFL - lastFL_enc;
        int deltaFR = currFR - lastFR_enc;
        int deltaRL = currBL - lastBL_enc;
        int deltaRR = currBR - lastBR_enc;

        double deltaSFL = deltaFL * EncodertoCm;
        double deltaSFR = deltaFR * EncodertoCm;
        double deltaSRL = deltaRL * EncodertoCm;
        double deltaSRR = deltaRR * EncodertoCm;

        double vx = (deltaSFL + deltaSFR + deltaSRL + deltaSRR) / (4 * dt);
        double vy = (deltaSFL - deltaSFR - deltaSRL + deltaSRR) / (4 * dt);
        double omega = (-deltaSFL + deltaSFR - deltaSRL + deltaSRR) / (4 * D * dt);

        double deltaTheta = omega * dt;
        double deltaX = (vx * Math.cos(theta) - vy * Math.sin(theta)) * dt;
        double deltaY = (vx * Math.sin(theta) + vy * Math.cos(theta)) * dt;

        x += deltaX;
        y += deltaY;
        theta += deltaTheta;
        theta = normalizeAngle(theta);

        lastTime = currentTime;
        lastFL_enc = currFL;
        lastFR_enc = currFR;
        lastBL_enc = currBL;
        lastBR_enc = currBR;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // 访问器
    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }

    public void resetPosition(){
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }

    public void ChassisLocationTelemetry(){
        telemetry.addData("X Position (cm): ", x);
        telemetry.addData("Y Position (cm): ", y);
        telemetry.addData("Heading (rad): ", theta);
    }

    public void ChassisVelocityTelemetry(){
        telemetry.addData("Front Left Velocity (cm/s): ", frontLeft.getVelocity() * EncodertoCm);
        telemetry.addData("Front Right Velocity (cm/s): ", frontRight.getVelocity() * EncodertoCm);
        telemetry.addData("Back Left Velocity (cm/s): ", backLeft.getVelocity() * EncodertoCm);
        telemetry.addData("Back Right Velocity (cm/s): ", backRight.getVelocity() * EncodertoCm);
    }
}