package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.HypParams;
import org.firstinspires.ftc.teamcode.utility.filter.EMA;
@Config
public class RobotPosition {
    private static final ConvexPolygon SHOOTING_AREA_LEFT = HypParams.SHOOTING_AREA_LEFT;
    private static final ConvexPolygon SHOOTING_AREA_RIGHT = HypParams.SHOOTING_AREA_RIGHT;
    private static final ConvexPolygon BoundingBox = HypParams.BoundingBox;

    static MecanumDrive drive;
    HardwareMap hardwareMap;
    Localizer localizer;
    public NormalizedColorSensor fullSensor;
    public NormalizedColorSensor emptySensor;

    // 对HSV三通道分别做EMA滤波，避免单帧噪声导致误判
    private final EMA hueFilter = new EMA(HypParams.ColorAlpha);
    private final EMA saturationFilter = new EMA(HypParams.ColorAlpha);
    private final EMA valueFilter = new EMA(HypParams.ColorAlpha);
    private final float[] filteredHsv = new float[3];

    public boolean ableToShoot = false;
    //todo :调整距离
    /**
     *true 表示有3个球
     * false 表示3个球未满（未检测到球）
     */
    public boolean isFull(){
        if (fullSensor == null) return false;
        readAndFilter(fullSensor, filteredHsv);
        return HypParams.isBall_Full(filteredHsv);
    }
    /**
     *true 表示无球
     *false 表示有球
     */
    public boolean isEmpty(){
        if (emptySensor == null) return true;
        readAndFilter(emptySensor, filteredHsv);
        return !(HypParams.isBall_Empty(filteredHsv));
    }

    /**
     * 读取颜色传感器，转为HSV并做EMA滤波，结果写入outHsv
     */
    private void readAndFilter(NormalizedColorSensor sensor, float[] outHsv) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] rawHsv = new float[3];
        Color.colorToHSV(colors.toColor(), rawHsv);

        if (!Float.isNaN(rawHsv[0]) && !Float.isNaN(rawHsv[1]) && !Float.isNaN(rawHsv[2])) {
            outHsv[0] = (float) hueFilter.update(rawHsv[0]);
            outHsv[1] = (float) saturationFilter.update(rawHsv[1]);
            outHsv[2] = (float) valueFilter.update(rawHsv[2]);
        }
    }

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

    // 位姿修正偏移量，用于消除累计误差
    private Pose2d poseCorrection = new Pose2d(0, 0, 0);
    private boolean correctionActive = false;

    private static RobotPosition instance;

    public static RobotPosition getInstance(){
        if(instance==null){
            throw new IllegalStateException("RobotPosition not initialized, call setInstance first");
        }
        return instance;
    }
    private RobotPosition(){
    }

;
    public static RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose) {

        instance=new RobotPosition();
        instance.hardwareMap = hardwareMap;

        instance.currentPose = initpose != null ? initpose : new Pose2d(0,0,0);
        instance.drive=new MecanumDrive(hardwareMap,instance.currentPose);
        instance.fullSensor = hardwareMap.get(NormalizedColorSensor.class, "FullSensor");
        instance.emptySensor = hardwareMap.get(NormalizedColorSensor.class, "EmptySensor");
        instance.localizer=instance.drive.localizer;
        return instance;
    }

    /**
     * 立即将机器人位姿修正为目标值，并保持该修正偏移量不变
     * 用于消除 pinpoint 累计误差。调用后，后续 update() 返回的位姿
     * 将始终基于本次修正偏移量计算，直到下一次 ResetPoseTo 被调用。
     *
     * @param pose 目标位姿（真实位姿）
     */
    public void ResetPoseTo(Pose2d pose) {
        Pose2d rawPose = localizer.getPose();
        // 计算修正偏移量 = 目标位姿 - 当前原始读数
        double dx = pose.position.x - rawPose.position.x;
        double dy = pose.position.y - rawPose.position.y;
        double dtheta = pose.heading.toDouble() - rawPose.heading.toDouble();
        poseCorrection = new Pose2d(dx, dy, dtheta);
        correctionActive = true;
        // 立即更新当前位姿
        currentPose = pose;
    }

    // 每帧调用：更新定位器并返回当前位姿
    public Pose2d update() {
        currentVelocity2d = drive.updatePoseEstimate();


        if (instance.localizer != null) {
            try {
                instance.localizer.update();
                Pose2d p = instance.localizer.getPose();
                if (p != null) {
                    // 应用位姿修正偏移（消除累计误差）
                    if (correctionActive) {
                        p = new Pose2d(
                                p.position.x + poseCorrection.position.x,
                                p.position.y + poseCorrection.position.y,
                                p.heading.toDouble() + poseCorrection.heading.toDouble()
                        );
                    }
                    instance.currentPose = p;
                }
            } catch (Exception ignored) {
                // 如果 localizer 的方法抛异常，保持现有 pose
            }
        }
        org.firstinspires.ftc.teamcode.utility.Point2D pose = new org.firstinspires.ftc.teamcode.utility.Point2D(instance.getX(), instance.getY());
        //ableToShoot = SHOOTING_AREA_LEFT.Contains(pose) || SHOOTING_AREA_RIGHT.Contains(pose); //基准点判断法
        ableToShoot = BoundingBox.inAbsolute(currentPose).IsIntersected(SHOOTING_AREA_LEFT) || BoundingBox.inAbsolute(currentPose).IsIntersected(SHOOTING_AREA_RIGHT); //碰撞框压线判断法
        return instance.currentPose;

    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    public double getVx(){
        double vxField = currentVelocity2d.linearVel.x;
        double vyField = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return vxField * Math.cos(theta) + vyField * Math.sin(theta);
    }
    public double getVy(){
        double vxField = currentVelocity2d.linearVel.x;
        double vyField = currentVelocity2d.linearVel.y;
        double theta = getTheta();
        return -vxField * Math.sin(theta) + vyField * Math.cos(theta);
    }
    public MecanumDrive getDrive(){return drive;}
    public double getOmega(){return currentVelocity2d.angVel;}
    public boolean isAbleToShoot(){return ableToShoot;}


}
