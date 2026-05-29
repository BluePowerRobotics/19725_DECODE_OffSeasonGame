package org.firstinspires.ftc.teamcode.Controllers.Sweeper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utility.HypParams;

@Config
public class Sweeper {
    //TODO 改成速度闭环/基于电压输出的开环
    public DcMotorEx motor;

    private Telemetry telemetry;

    private int targetVelocity = 0;

    public static int ForR = 0;

    private boolean isPreparing = false;
    private int prepareStartPos = 0;
    private int prepareTargetTicks = 0;

    public Sweeper(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.motor = hardwareMap.get(DcMotorEx.class, "sweeperMotor");
        setDirection();
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void setDirection() {
        switch (ForR) {
            case 0:
                motor.setDirection(DcMotor.Direction.REVERSE);
                break;
            case 1:
                motor.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }

    public void setEat() {
        targetVelocity = HypParams.SweeperEatVel;
    }

    public void setGiveArtifact() {
        targetVelocity = HypParams.SweeperGiveArtifactVel;
    }

    public void setOutput() {
        targetVelocity = HypParams.SweeperOutputVel;
    }

    public void setStop() {
        targetVelocity = 0;
    }

    public void setTrigger() {
        targetVelocity = HypParams.SweeperTriggerVel;
    }

    /**
     * 发射前准备：反转sweeper将球拉离飞轮
     * 反转速度与吐球速度相同
     *
     * @param ticks 反转ticks数
     */
    public void prepare(int ticks) {
        if (!isPreparing) {
            isPreparing = true;
            prepareTargetTicks = ticks;
            prepareStartPos = motor.getCurrentPosition();
            targetVelocity = HypParams.SweeperOutputVel;
        }
    }

    /**
     * 检查sweeper准备阶段是否完成
     */
    public boolean isPreparing() {
        return isPreparing;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void update() {
        if (isPreparing) {
            int traveled = Math.abs(motor.getCurrentPosition() - prepareStartPos);
            if (traveled >= prepareTargetTicks) {
                isPreparing = false;
                targetVelocity = 0;
            }
        }
        motor.setVelocity(targetVelocity);
    }
}

