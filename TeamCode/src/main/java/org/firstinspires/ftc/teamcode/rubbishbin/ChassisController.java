package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDController;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class ChassisController {
    public static class Params{
        //todo 调整参数
        public double maxV=0.5; // 最大线速度 (m/s)
        public double maxOmega=Math.PI*1/2; // 最大角速度 (rad/s)
        public double zeroThresholdV =0.05; // 速度零点阈值 (m/s)
        public double zeroThresholdOmega =Math.toRadians(0.5); // 角速度零点阈值 (rad/s)
        public double FrontToCenterInch=9.75;
        public double BackToCenterInch=0;
        public double SideToCenterInch=7;
    }
    public static ChassisController_VER2.Params PARAMS = new ChassisController_VER2.Params();
    HardwareMap hardwareMap;
    RobotPosition robotPosition;
    Telemetry telemetry;
    Pose2d currentPose=new Pose2d(0,0,0);

    MecanumDrive drive;
    boolean useNoHeadMode=false;
    double HeadingLockRadian;
    public double getHeadingLockRadian(){return HeadingLockRadian;}
    public double noHeadModeStartError;





    /**
    构造函数
     */
    public ChassisController(HardwareMap hardwareMap, Telemetry telemetry1){
        this.hardwareMap=hardwareMap;
        this.telemetry=telemetry1;
        robotPosition = RobotPosition.RobotPositioninit(hardwareMap,currentPose);
        drive=robotPosition.drive;
        HeadingLockRadian = RobotPosition.getInstance().getTheta();
        noHeadModeStartError=RobotPosition.getInstance().getTheta();

    }

    /**
     *
     * @param x 手柄输入x
     * @param y 手柄输入y
     * @param theta     手柄输入旋转
      *
     */
    public void update(double x,double y,double theta){
        double vx=x*PARAMS.maxV;
        double vy=y*PARAMS.maxV;
        double omega=theta*PARAMS.maxOmega;
        if (useNoHeadMode)
            solveGround(vx, vy, omega, RobotPosition.getInstance().getTheta()-noHeadModeStartError);
        else
            solveChassis(vx, vy, omega);


    }
    public void exchangeNoHeadMode(){
        useNoHeadMode=!useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    public static class Params_Calculating {
        //todo 调整参数
        public double rkP = 0.8;//radian k
        public double rkI = 1;
        public double rkD = 0.1;
        public double drkP=0.2;
        public double drkI=0;
        public double drkD=0.05;
        public double powerfulPIDRadianUseTimeMS = 1000;
    }
    public static ChassisController_VER2.Params_Calculating params_calculating = new ChassisController_VER2.Params_Calculating();
    long lastHeadingSetTimeMS=0;
    PIDController pidRadianHeadLock= new PIDController(params_calculating.rkP, params_calculating.rkI, params_calculating.rkD);
    PIDController pidRadianDrive= new PIDController(params_calculating.drkP, params_calculating.drkI, params_calculating.drkD);
    /**
     * @param vx    机器人相对于自身的横移速度 (m/s) —— +右
     * @param vy    机器人相对于自身的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     */
    public void solveChassis(double vx, double vy, double omega) {
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(vy,-vx),
                omega
        ));
    }

    /**

     * 逆运动学公式（地面坐标系）
     * @param vx 机器人相对于地面的横移速度 (m/s) —— +右
     * @param vy 机器人相对于地面的前进速度 (m/s) —— +前
     * @param omega 机器人旋转角速度 (rad/s) —— +逆时针
     * @param headingRadian 机器人朝向，弧度制
     */
    public void solveGround(double vx, double vy, double omega, double headingRadian) {

        double vxPro = vx * Math.cos(headingRadian) + vy * Math.sin(headingRadian);
        double vyPro = -vx * Math.sin(headingRadian) + vy * Math.cos(headingRadian);
        solveChassis(vxPro, vyPro, omega);
    }
    public void telemetry(){
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("Use No Head Mode",useNoHeadMode);
        telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());

    }


}
