package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.PIDController;

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
    public static Params PARAMS = new Params();

    Pose2d initialPose=new Pose2d(0,0,0);
    Pose2d currentPose=new Pose2d(0,0,0);
    HardwareMap hardwareMap;
    MecanumDrive drive;
    boolean useNoHeadMode=false;
    public boolean runningToPoint=false;
//    boolean autoLockHeading=true;
    boolean HeadingLockRadianReset=true;
    double HeadingLockRadian;
    public double getHeadingLockRadian(){return HeadingLockRadian;}
    public double noHeadModeStartError;


    public ChassisController(HardwareMap hardwareMap,Pose2d initialPose){
        this.hardwareMap=hardwareMap;
        this.initialPose=initialPose;
        drive=new MecanumDrive(hardwareMap,initialPose);
        RobotPosition.getInstance().RobotPositioninit(hardwareMap,initialPose,drive.localizer);
        HeadingLockRadian = RobotPosition.getInstance().getHeading();
        noHeadModeStartError=RobotPosition.getInstance().getHeading();
    }
    public void gamepadInput(double vx,double vy,double omega){
        vx=vx*PARAMS.maxV;
        vy=vy*PARAMS.maxV;
        omega=omega*PARAMS.maxOmega;
        if(!runningToPoint) {
//            if(autoLockHeading){
//                if(omega!=0){
//                    HeadingLockRadianReset=true;
//                }else{
//                    if(HeadingLockRadianReset){
//                        HeadingLockRadianReset=false;
//                        firstRunRadian=true;
//                        HeadingLockRadian=robotPosition.getHeading();
//                    }
//                    if(Math.abs(robotPosition.getHeading()-HeadingLockRadian)<= PARAMS.zeroThresholdOmega) {
//                        pidRadianHeadLock.reset();
//                    }
//                    omega=calculatePIDRadian(HeadingLockRadian,robotPosition.getHeading());
//                }
//            }
            if (useNoHeadMode)
                solveGround(vx, vy, omega, RobotPosition.getInstance().getHeading()-noHeadModeStartError);
            else
                solveChassis(vx, vy, omega);
        }
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
    public static Params_Calculating params_calculating = new Params_Calculating();
    long lastHeadingSetTimeMS=0;
    PIDController pidRadianHeadLock= new PIDController(params_calculating.rkP, params_calculating.rkI, params_calculating.rkD);
    PIDController pidRadianDrive= new PIDController(params_calculating.drkP, params_calculating.drkI, params_calculating.drkD);


    /**
     *
     *
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
    public void solveGround(double[] vxy,double vOmega,double headingRadian){
        solveGround(vxy[0],vxy[1],vOmega,headingRadian);
    }


    long lastTimeRadian = 0;
    boolean firstRunRadian = true;

    public double calculatePIDRadian(double targetRadian, double currentRadian) {
        if (firstRunRadian) {
            firstRunRadian = false;
            lastTimeRadian = System.nanoTime();
            pidRadianDrive.reset();
        }
        double errorRadian = targetRadian - currentRadian;
        // 归一化到[-π, π]
        errorRadian = MathSolver.normalizeAngle(errorRadian);

        if(System.currentTimeMillis()-lastHeadingSetTimeMS>= params_calculating.powerfulPIDRadianUseTimeMS){
            pidRadianDrive.setPID(params_calculating.drkP, params_calculating.drkI, params_calculating.drkD);
            double output = pidRadianDrive.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
            lastTimeRadian = System.nanoTime();
            return output;
        }
        pidRadianHeadLock.setPID(params_calculating.rkP, params_calculating.rkI, params_calculating.rkD);
        double output = pidRadianHeadLock.calculate(errorRadian, 0, (System.nanoTime() - lastTimeRadian) / 1e9);
        lastTimeRadian = System.nanoTime();
        return output;
    }
}

