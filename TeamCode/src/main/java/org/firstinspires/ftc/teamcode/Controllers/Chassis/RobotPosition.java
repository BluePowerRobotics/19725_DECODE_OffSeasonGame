package org.firstinspires.ftc.teamcode.Controllers.Chassis;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/*
 简短说明：
 - 支持通过枚举选择定位器类型（目前支持 MECANUM）或直接传入实现了 Localizer 的实例。
 - init() 将初始位姿设置到定位器（若定位器支持）。
 - update() 每帧调用，内部调用 localizer.update() 并读取当前位姿。
 - 提供 getPose/getX/getY/getHeading 访问当前位姿。
*/

public class RobotPosition {
    private static RobotPosition instance; // 单例实例

    /**
     * 获取RobotPosition实例
     * @return RobotPosition实例
     * @throws IllegalStateException 如果实例未初始化
     */
    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }



    HardwareMap hardwareMap;
    private Localizer localizer;
    public double inchPerTick = 1;

    private Pose2d initpose;
    private Pose2d pose2d;



    // 构造：直接传入 Localizer 实例（外部创建特定实现）
    public void RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose, Localizer localizer) {
        this.hardwareMap = hardwareMap;
        this.initpose = initpose != null ? initpose : new Pose2d(0,0,0);
        this.pose2d = this.initpose;
        this.localizer = localizer;
    }

    // init：将初始位姿设置到 localizer（如果支持 setPoseEstimate）
    public void init() {
        if (localizer != null) {
            try {
                localizer.setPose(initpose);
            } catch (Exception ignored) {
                // 若 localizer 不支持 setPoseEstimate，则忽略
            }
        }
        this.pose2d = this.initpose;
    }

    // 每帧调用：更新定位器并返回当前位姿
    public Pose2d update() {
        if (localizer != null) {
            try {
                localizer.update();
                Pose2d p = localizer.getPose();
                if (p != null) {
                    this.pose2d = p;
                }
            } catch (Exception ignored) {
                // 如果 localizer 的方法抛异常，保持现有 pose
            }
        }
        return this.pose2d;
    }

    public Pose2d getPose() {
        return this.pose2d;
    }

    public double getX() {
        return this.pose2d.position.x;
    }

    public double getY() {
        return this.pose2d.position.y;
    }

    public double getHeading() {
        return this.pose2d.heading.toDouble();
    }

}