package org.firstinspires.ftc.teamcode.Controllers.MotorExamples;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.Line;
import org.firstinspires.ftc.teamcode.utility.MathSolver;
import org.firstinspires.ftc.teamcode.utility.Point2D;
import org.firstinspires.ftc.teamcode.utility.filter.MeanFilter;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * MotorSVATuning 类用于自动或手动调优电机的 SVA 参数
 * SVA 参数包括：kS（静态摩擦系数）、kV（速度系数）、kA（加速度系数）
 */
@Config
@TeleOp(name = "Motor SVA Tuning")
public class MotorSVATuning extends LinearOpMode {
    /** 电机实例 */
    private DcMotorEx motor;
    private DcMotorEx followerMotor;
    /** 文件写入器，用于记录调优数据 */
    private FileWriter fileWriter;
    /** 日志文件名 */
    private String logFileName;
    /** 电压输出控制器 */
    private VoltageOut voltageOut;
    /** 上次A按钮状态 */
    private boolean lastA = false;

    /** 调优模式枚举 */
    public enum TuningMode {
        AUTOMATIC, MANUAL;
        
        /**
         * 切换到下一个模式
         * @return 下一个模式
         */
        TuningMode next() {
            return this == AUTOMATIC ? MANUAL : AUTOMATIC;
        }
    }

    /** 调优状态枚举 */
    enum State {
        kS_ASSESSING,     // 评估kS
        kS_kV_FITTING,    // 拟合kS和kV
        WAITING,          // 等待电机停止
        kA_ASSESSING,     // 评估kA
        FINISHED          // 调优完成
    }

    /** 当前调优状态 */
    State state = State.kS_kV_FITTING;
    /** 用于拟合kS和kV的数据点 */
    List<Point2D> points_kS_kV = new ArrayList<>();
    /** 用于拟合kA的数据点 */
    List<Point2D> points_kA = new ArrayList<>();
    /** 均值滤波器，用于平滑速度数据 */
    MeanFilter meanFilter = new MeanFilter(20);

    /** 静态摩擦系数 */
    public static double kS = 0;
    /** 速度系数 */
    public static double kV = 0;
    /** 加速度系数 */
    public static double kA = 0;

    // 可配置参数
    /** 调优模式 */
    public static TuningMode tuningMode = TuningMode.MANUAL;
    /** kS评估的电压增量 */
    public static double KS_VOLTAGE_INCREMENT = 1;
    /** 判定电机开始运动的速度阈值 */
    public static double KS_VELOCITY_THRESHOLD = 20; 
    /** 测试点数量 */
    public static int testPoints = 5;
    /** 最大方差，用于判定速度是否稳定 */
    public static double maxVariance = 1;
    /** kA测试电压数组 */
    public static double[] kA_TestVoltages = new double[]{1, 2, 3, 4, 5, 6};
    /** 手动模式下的电压增量 */
    public static double voltageIncrement = 1;
    /** 电机名称 */
    public static String motorName = "motorL";

    @Override
    public void runOpMode() throws InterruptedException {
        // 初始化遥测，同时输出到Driver Station和Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 初始化电机
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        followerMotor = hardwareMap.get(DcMotorEx.class, "motorR"); // 假设有第二个电机作为跟随
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        followerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化电压输出控制器
        voltageOut = new VoltageOut(hardwareMap);

        // 初始化日志文件
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.CHINA);
        logFileName = Environment.getExternalStorageDirectory().getPath() + "/" + sdf.format(new Date()) + "_MotorSVA.csv";
        try {
            fileWriter = new FileWriter(logFileName);
            fileWriter.write("Time,OutputVoltage,Velocity\n");
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to create log file: " + e.getMessage());
            telemetry.update();
            return;
        }

        // 初始化阶段，切换调优模式
        while (opModeInInit()) {
            telemetry.addLine("Tuning Mode: " + tuningMode);
            telemetry.addLine("Press A to switch mode");
            if (gamepad1.aWasPressed()) { // 假设aWasReleased逻辑，此处简化
                tuningMode = tuningMode.next();
            }
            telemetry.update();
        }

        waitForStart();

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 调优变量
        double outputVoltage = 0;
        int currentTestPoint = 0;
        int kA_TestIndex = 0;
        boolean kA_Started = false;
        long lastTime = System.nanoTime();
        double lastVelocity = 0;
        List<Double> accelerations = new ArrayList<>();

        // 主循环
        while (opModeIsActive()) {
            // 获取当前电机速度
            double velocity = motor.getVelocity(); // 假设getVelocity()返回ticks per second或类似单位
            
            // 计算时间间隔和加速度
            long nowTime = System.nanoTime();
            double deltaTime = (nowTime - lastTime) / 1e9;
            double acceleration = (velocity - lastVelocity) / deltaTime;
            lastVelocity = velocity;
            lastTime = nowTime;

            // 根据当前状态执行不同的调优步骤
            switch (state) {
                case kS_ASSESSING:
                    // 缓慢增加输出电压
                    outputVoltage += KS_VOLTAGE_INCREMENT / 100;//延缓增加，给电流作用的时间
                    // 当电机开始运动时，记录当前电压为kS
                    if (Math.abs(velocity) > KS_VELOCITY_THRESHOLD) { // Threshold to start moving
                        kS = outputVoltage;
                        state = State.kS_kV_FITTING;
                    }
                    break;
                case kS_kV_FITTING:
                    if (tuningMode == TuningMode.AUTOMATIC) {
                        // 自动模式：收集稳定速度数据点
                        if (currentTestPoint < testPoints) {
                            meanFilter.filter(velocity);
                            // 当速度稳定时记录数据点
                            if (meanFilter.getCount() >= meanFilter.getWindowSize() && meanFilter.getVariance() <= maxVariance) {
                                points_kS_kV.add(new Point2D(outputVoltage, meanFilter.getMean()));
                                currentTestPoint++;
                                meanFilter.reset();
                                // 计算下一个测试点的电压
                                outputVoltage += (voltageOut.getVoltage() - kS) / testPoints;
                            }
                        } else {
                            // 拟合直线计算kV和kS
                            Line line = MathSolver.fitLine(points_kS_kV);
                            kV = 1 / line.getSlope();
                            kS = line.getXIntercept();
                            state = State.WAITING;
                        }
                    } else { // MANUAL
                        // 手动模式：通过游戏手柄控制电压
                        if (gamepad1.dpadUpWasPressed()) {
                            outputVoltage += voltageIncrement;
                        } else if (gamepad1.dpadDownWasPressed()) {
                            outputVoltage -= voltageIncrement;
                        }
                        if(gamepad1.dpadLeftWasPressed()){
                            outputVoltage -= KS_VOLTAGE_INCREMENT;
                        }
                        if(gamepad1.dpadRightWasPressed()){
                            outputVoltage += KS_VOLTAGE_INCREMENT;
                        }
                        // 按A键记录数据点
                        if (gamepad1.a) { // Record point
                            points_kS_kV.add(new Point2D(outputVoltage, velocity));
                            currentTestPoint++;
                            if (currentTestPoint >= testPoints) {
                                // 拟合直线计算kV和kS
                                Line line = MathSolver.fitLine(points_kS_kV);
                                kV = 1 / line.getSlope();
                                kS = line.getXIntercept();
                                //state = State.WAITING;
                            }
                        }
                        // 按Start键结束调优
                        if(gamepad1.start){
                            //state = State.WAITING;
                            state = State.FINISHED;
                        }
                    }
                    break;
                case WAITING:
                    // 停止电机，等待其完全停止
                    motor.setPower(0);
                    if (Math.abs(velocity) < 10) {
                        state = State.kA_ASSESSING;
                    }
                    break;
                case kA_ASSESSING:
                    // 开始kA评估
                    if (!kA_Started) {
                        accelerations.clear();
                        lastTime = System.nanoTime();
                        lastVelocity = velocity;
                        kA_Started = true;
                    }
                    // 记录加速度数据
                    accelerations.add(acceleration);
                    // 计算加速电压
                    double speedUpVoltage = kA_TestVoltages[kA_TestIndex];
                    outputVoltage = kS + kV * velocity + speedUpVoltage;
                    // 当输出电压超过电池电压时，记录数据点
                    if (outputVoltage > voltageOut.getVoltage()) {
                        double avgAccel = MathSolver.avg(accelerations.toArray(new Double[0]));
                        points_kA.add(new Point2D(avgAccel, speedUpVoltage));
                        kA_TestIndex++;
                        kA_Started = false;
                        // 所有测试点完成后，拟合直线计算kA
                        if (kA_TestIndex >= kA_TestVoltages.length) {
                            Line line = MathSolver.fitLine(points_kA);
                            kA = 1 / line.getSlope();
                            state = State.FINISHED;
                        }
                    }
                    break;
                case FINISHED:
                    // 调优完成，停止输出
                    outputVoltage = 0;
                    break;
            }
            // 设置电机功率
            motor.setPower(-voltageOut.getVoltageOutPower(outputVoltage));
            followerMotor.setPower(-voltageOut.getVoltageOutPower(outputVoltage)); // 同步跟随电机

            // 输出遥测数据
            telemetry.addData("State", state);
            telemetry.addData("Output Voltage", outputVoltage);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addData("kA", kA);
            telemetry.update();

            // 记录数据到文件
            try {
                fileWriter.write(System.currentTimeMillis() + "," + outputVoltage + "," + velocity + "\n");
            } catch (IOException e) {
                telemetry.addData("Log Error", e.getMessage());
            }

            sleep(10); // 小延迟
        }

        // 关闭文件
        try {
            fileWriter.close();
        } catch (IOException e) {
            telemetry.addData("Close Error", e.getMessage());
        }
    }
}

// Note: Point2D class assumed to be available from utility package

