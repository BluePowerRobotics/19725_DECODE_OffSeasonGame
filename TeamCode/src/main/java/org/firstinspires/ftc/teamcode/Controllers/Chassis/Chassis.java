 package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.OffseasonDECODE;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.Controllers.MotorExamples.PIDSVAControllers.PIDController;


@Config
public class Chassis {
    private static final ConvexPolygon SHOOTING_AREA_LEFT = HypParams.SHOOTING_AREA_LEFT;
    private static final ConvexPolygon SHOOTING_AREA_RIGHT = HypParams.SHOOTING_AREA_RIGHT;

    private final double maxV = HypParams.maxV;
    private final double maxOmega = HypParams.maxOmega;
    private double wanderSpeed = HypParams.WanderSpeed;

    private final MecanumDrive drive;
    private final ActionRunner actionRunner;
    private boolean useNoHeadMode = HypParams.InitialUseNoHeadMode;
    private final Telemetry telemetry;
    private double lastKx = 0, lastKy = 0, lastKomega = 0;

    // 航向PID控制器，用于GoTo方法的旋转功率控制
    private final PIDController headingPID;
    private double lastGoToTime = 0;
    private double lastNormalizedError = 0; // 上一次的角度误差，用于检测±π跳变

    public Chassis(HardwareMap hardwareMap, OffseasonDECODE.TEAM_COLOR teamColor, ActionRunner actionRunner, Telemetry telemetry) {
        Pose2d startPose = (teamColor == OffseasonDECODE.TEAM_COLOR.RED) ?
                HypParams.startPoseRed : HypParams.startPoseBlue;
        RobotPosition.RobotPositioninit(hardwareMap, startPose);
        this.drive = RobotPosition.getInstance().getDrive();
        this.actionRunner = actionRunner;
        this.telemetry = telemetry;
        // 初始化航向PID：kP=3.0, kI=0.1, kD=0.05, maxI=0.5
        this.headingPID = new PIDController(3.0, 0.1, 0.05, 0.5);

    }

    public void setUseNoHeadMode(boolean useNoHeadMode){
        this.useNoHeadMode = useNoHeadMode;
    }
    public void exchangeUseNoHeadMode(){
        useNoHeadMode = !useNoHeadMode;
    }
    public boolean getUseNoHeadMode(){
        return useNoHeadMode;
    }
    public void stop(){
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0,0),
                0));
    }
    public void GoTo(double targetTheta){ //targetTheta表示目标方向与小车正前方（x轴正方向）的夹角（逆时针为正）
        // 将角度差归一化到[-π, π]
        double error = MathSolver.normalizeAngle(targetTheta);

        // 误差连续性保护：避免相邻帧在±π边界跳变导致D项尖峰
        double errorDelta = error - lastNormalizedError;
        if (errorDelta > Math.PI) {
            error -= 2 * Math.PI;
        } else if (errorDelta < -Math.PI) {
            error += 2 * Math.PI;
        }
        lastNormalizedError = error;

        // 速度分量：向目标方向前进
        double vx = Math.cos(targetTheta) * wanderSpeed;
        double vy = Math.sin(targetTheta) * wanderSpeed;

        // 计算时间间隔（秒）
        double currentTime = System.nanoTime() / 1e9;
        double dt = (lastGoToTime > 0) ? (currentTime - lastGoToTime) : 0.02;
        if (dt <= 0 || dt > 0.5) dt = 0.02; // 防止异常时间间隔
        lastGoToTime = currentTime;

        // 使用标准PID语义：setpoint=0（目标偏差为0），measurement为当前偏差的负值
        // calculate(setpoint, measurement, dt) 内部: error = setpoint - measurement = 0 - (-error) = error
        double omega = headingPID.calculate(0, -error, dt);
        // 角速度限幅
        omega = Math.max(-maxOmega, Math.min(maxOmega, omega));

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(vx,vy),
                omega));

    }
    public void GoToShootingArea(){
        Point2D currentPos = new Point2D(
            RobotPosition.getInstance().getX(),
            RobotPosition.getInstance().getY()
        );

        if (RobotPosition.getInstance().isAbleToShoot()) {
            stop();
            return;
        }
        //前往最近的发射区，可以改为只去小三角
        Point2D VecToLeft = SHOOTING_AREA_LEFT.NearestVectorFrom(currentPos);
        Point2D VecToRight = SHOOTING_AREA_RIGHT.NearestVectorFrom(currentPos);

        Point2D nearestVector = (HypParams.ToLeft) ? VecToLeft : VecToRight;
        double targetTheta = nearestVector.getRadian()-RobotPosition.getInstance().getTheta();

        GoTo(targetTheta);
    }


    /**
     * 设置航向PID参数（用于手柄热调参）
     */
    public void setHeadingPID(double kP, double kI, double kD) {
        headingPID.setPID(kP, kI, kD);
    }

    /**
     * 重置航向PID积分与微分状态
     */
    public void resetHeadingPID() {
        headingPID.reset();
    }

    /** @return 航向PID的kP值 */
    public double getHeadingKP() { return headingPID.getKP(); }
    /** @return 航向PID的kI值 */
    public double getHeadingKI() { return headingPID.getKI(); }
    /** @return 航向PID的kD值 */
    public double getHeadingKD() { return headingPID.getKD(); }

    /** 设置漫游速度（用于手柄调节） */
    public void setWanderSpeed(double speed) {
        this.wanderSpeed = Math.max(0, Math.min(maxV, speed));
    }

    /** @return 当前漫游速度 */
    public double getWanderSpeed() { return wanderSpeed; }

    public void update(double Kx, double Ky, double Komega){
        lastKx = Kx;
        lastKy = Ky;
        lastKomega = Komega;
        if(!actionRunner.isBusy()){
            // 摇杆 → 底盘速度映射：Ky/Kx 取反以匹配 FTC SDK 手柄惯例（上推为负、右推为正）
            // 官方 SDK: forward = -gamepad1.left_stick_y, strafe = gamepad1.left_stick_x
            // Road Runner: PoseVelocity2d.y 正值 = 向左横移，故 strafe 也需取反
            double forwardVel = -Ky * maxV;
            double strafeVel = -Kx * maxV;
            double omega = -Komega * maxOmega;
            if(useNoHeadMode){
                double theta = RobotPosition.getInstance().getTheta();
                double cos = Math.cos(theta);
                double sin = Math.sin(theta);
                // 将场心坐标系速度旋转到机器人坐标系
                double forwardRobot = forwardVel * cos + strafeVel * sin;
                double strafeRobot = -forwardVel * sin + strafeVel * cos;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardRobot, strafeRobot), omega));
            }
            else{
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forwardVel, strafeVel), omega));
            }
        }
    }
    public void telemetry(){
        telemetry.addData("X",RobotPosition.getInstance().getX());
        telemetry.addData("Y",RobotPosition.getInstance().getY());
        telemetry.addData("Heading",Math.toDegrees(RobotPosition.getInstance().getTheta()));
        telemetry.addData("useNoHeadMode", useNoHeadMode);
        telemetry.addData("HeadingPID_kP", headingPID.getKP());
        telemetry.addData("HeadingPID_kI", headingPID.getKI());
        telemetry.addData("HeadingPID_kD", headingPID.getKD());
        telemetry.addData("lfP",drive.leftFront.getPower());
        telemetry.addData("rfP",drive.rightFront.getPower());
        telemetry.addData("lbP",drive.leftBack.getPower());
        telemetry.addData("rbP",drive.rightBack.getPower());

        //telemetry.addData("Vx",RobotPosition.getInstance().getVx());
        //telemetry.addData("Vy",RobotPosition.getInstance().getVy());
        //telemetry.addData("Omega",Math.toDegrees(RobotPosition.getInstance().getOmega()));
        telemetry.addData("lfV",drive.leftFront.getVelocity());
        telemetry.addData("rfV",drive.rightFront.getVelocity());
        telemetry.addData("lbV",drive.leftBack.getVelocity());
        telemetry.addData("rbV",drive.rightBack.getVelocity());
        telemetry.addData("LeftStickX", lastKx);
        telemetry.addData("LeftStickY", lastKy);
        telemetry.addData("RightStickX", lastKomega);

    }
}