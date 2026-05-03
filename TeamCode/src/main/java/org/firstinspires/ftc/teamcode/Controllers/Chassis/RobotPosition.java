package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Point2D;

public class RobotPosition {
    private static final ConvexPolygon SHOOTING_AREA_LEFT = new ConvexPolygon(
        new Point2D(-72, 12),
        new Point2D(-72, -12),
        new Point2D(-60, 0)
    );

    private static final ConvexPolygon SHOOTING_AREA_RIGHT = new ConvexPolygon(
        new Point2D(0, 0),
        new Point2D(72, 72),
        new Point2D(72, -72)
    );

    private static final ConvexPolygon BoundingBox = new ConvexPolygon(
        //小车碰撞框顶点的相对坐标，可以调整顶点数量，如有需要可以留一点余量
        new Point2D(20, 20),
        new Point2D(-20, 20),
        new Point2D(-20, -20),
        new Point2D(20, -20)
    );

    public ConvexPolygon getShootingAreaLeft(){
        return SHOOTING_AREA_LEFT;
    }
    public ConvexPolygon getShootingAreaRight(){
        return SHOOTING_AREA_RIGHT;
    }
    public ConvexPolygon getBoundingBox(){
        return BoundingBox;
    }

    static MecanumDrive drive;
    HardwareMap hardwareMap;
    Localizer localizer;
    public DistanceSensor sensorDistancemax;
    public DistanceSensor sensorDistancemin;
    public static double maxdistance=20;//此变量代表球满时与传感器的距离：最大值
    public static double mindistance=50;//此变量代表球空时与传感器的距离：最小值
    public static double wrongdistance=30;//那个b传感器，在3cm以内搁那乱转，神经病，写这个变量避免一下这种愚蠢行为。

    public boolean ableToShoot = false;
    //todo :调整距离
    /**
     *true 表示有3个球
     * false 表示3个球未满（未检测到球）
     */
    public boolean isFull(){
        double sensor_distance=sensorDistancemax.getDistance(DistanceUnit.MM);
        if (sensor_distance <= maxdistance && sensor_distance>=wrongdistance) {  //瞪大你的眼睛好好看看，这条件成立的了吗？20>=30?
            return true;
        }
        else{
            return false;
        }
    }
    /**
     *true 表示无球
     *false 表示有球
     */
    public boolean isEmpty(){
        double sensor_distance=sensorDistancemin.getDistance(DistanceUnit.MM);

        if (sensor_distance >= mindistance && sensor_distance>=wrongdistance) {
            return true;
        }
        else{
            return false;
        }
    }

    public Pose2d currentPose;
    public PoseVelocity2d currentVelocity2d;

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
        instance.sensorDistancemax = hardwareMap.get(DistanceSensor.class, "dismax");
        instance.sensorDistancemin = hardwareMap.get(DistanceSensor.class, "dismin");
        instance.localizer=instance.drive.localizer;
        return instance;
    }

    // 每帧调用：更新定位器并返回当前位姿
    public Pose2d update() {
        currentVelocity2d = drive.updatePoseEstimate();


        if (instance.localizer != null) {
            try {
                instance.localizer.update();
                Pose2d p = instance.localizer.getPose();
                if (p != null) {
                    instance.currentPose = p;
                }
            } catch (Exception ignored) {
                // 如果 localizer 的方法抛异常，保持现有 pose
            }
        }
        org.firstinspires.ftc.teamcode.utility.Point2D pose = new org.firstinspires.ftc.teamcode.utility.Point2D(instance.getX(), instance.getY());
        ableToShoot = SHOOTING_AREA_LEFT.Contains(pose) || SHOOTING_AREA_RIGHT.Contains(pose); //基准点判断法
        ableToShoot = BoundingBox.inAbsolute(currentPose).IsIntersected(SHOOTING_AREA_LEFT) || BoundingBox.inAbsolute(currentPose).IsIntersected(SHOOTING_AREA_RIGHT); //碰撞框压线判断法
        return instance.currentPose;

    }


    public Pose2d getPose2d(){        return currentPose;    }

    public double getX(){     return currentPose.position.x;    }
    public double getY(){   return currentPose.position.y;    }
    public double getTheta(){ return currentPose.heading.toDouble();    }
    public double getVx(){return  currentVelocity2d.linearVel.x;}
    public double getVy(){return  currentVelocity2d.linearVel.y;}
    public MecanumDrive getDrive(){return drive;}
    public double getOmega(){return currentVelocity2d.angVel;}
    public boolean isAbleToShoot(){return ableToShoot;}


}
