package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

/**
 * 机器人炮台瞄准计算工具类
 * 功能：更新目标位置、计算炮台需要的旋转角度（考虑炮台相对机器人中心的偏移）
 * 坐标系：平面直角坐标系（x水平向右，y垂直向上）
 * 角度单位：所有输入/输出均为【度】，内部自动转换为弧度计算
 * 旋转角规则：输出delta_theta∈[-180°,180°]，正=顺时针旋转，负=逆时针旋转（最短路径）
 */
@Config
public class TurretAimingCalculator {
    // 内置目标位置（世界坐标系，x/y）
    private double targetX;
    private double targetY;

    // 炮台相对机器人中心的偏移参数（核心调整项，根据硬件实际值设置，单位：与机器人位置同尺度）
    public double turretOffsetX; // 炮台在机器人坐标系x轴方向的偏移（正=右，负=左）
    public double turretOffsetY; // 炮台在机器人坐标系y轴方向的偏移（正=上，负=下）

    /**
     * 构造方法（默认炮台无偏移，机器人中心即为炮台中心）
     */


    // ---------------------- 第一个核心方法：更新内置目标位置 ----------------------
    /**
     * 构造函数，输入目标世界坐标，更新工具类内置的目标定位
     * @param targetX 目标在世界坐标系的x坐标
     * @param targetY 目标在世界坐标系的y坐标
     */
    public TurretAimingCalculator(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }
    /**
     * 输入目标世界坐标，更新工具类内置的目标定位
     * @param targetX 目标在世界坐标系的x坐标
     * @param targetY 目标在世界坐标系的y坐标
     */
    public void setTargetPosition(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
    }

    // ---------------------- 第二个核心方法：计算炮台旋转角 ----------------------
    /**
     * 输入机器人状态，计算炮台需要旋转的角度（delta_theta）
     * @param robotX 机器人中心在世界坐标系的x坐标
     * @param robotY 机器人中心在世界坐标系的y坐标
     * @param theta 机器人自身方向角（度）：机器人前进方向与x轴正方向的夹角，逆时针为正
     * @param thetaTurret 炮台相对机器人的朝向角（度）：炮台与机器人前进方向的夹角，逆时针为正
     * @return delta_theta 炮台旋转角（度）：∈[-180°,180°]，正=顺时针，负=逆时针（最短路径）
     */
    public double calculateTurretRotateAngle(double robotX, double robotY, double theta, double thetaTurret) {
        // 1. 将角度转换为弧度（Java数学库Math.atan2/cos/sin均使用弧度）
        double thetaRad = Math.toRadians(theta);
        double thetaTurretRad = Math.toRadians(thetaTurret);
        // 目标位置（已内置，直接使用）
        double tarX = this.targetX;
        double tarY = this.targetY;

        // 2. 计算【炮台在世界坐标系的实际坐标】（考虑机器人方向和炮台偏移）
        // 偏移量随机器人方向旋转的坐标变换公式（机器人方向为theta，偏移量绕机器人中心旋转theta）
        double turretWorldX = robotX + turretOffsetX * Math.cos(thetaRad) - turretOffsetY * Math.sin(thetaRad);
        double turretWorldY = robotY + turretOffsetX * Math.sin(thetaRad) + turretOffsetY * Math.cos(thetaRad);

        // 3. 计算【目标相对于炮台的位置矢量】（世界坐标系）
        double deltaX = tarX - turretWorldX; // 目标在炮台右侧为正，左侧为负
        double deltaY = tarY - turretWorldY; // 目标在炮台上侧为正，下侧为负

        // 4. 计算炮台【需要的绝对瞄准角】（世界坐标系，弧度）：炮台指向目标的方向与x轴正方向的夹角
        // Math.atan2(dy, dx)：返回[-π, π]的弧度值，自动处理四象限，完美适配平面坐标角度计算
        double aimAngleRad = Math.atan2(deltaY, deltaX);

        // 5. 计算炮台【当前的绝对朝向角】（世界坐标系，弧度）：机器人方向 + 炮台相对机器人朝向
        double currentTurretAngleRad = thetaRad + thetaTurretRad;

        // 6. 计算炮台【原始旋转角】（弧度）：需要的角度 - 当前的角度
        double rotateRad = aimAngleRad - currentTurretAngleRad;

        // 7. 将旋转角归一化到[-π, π]弧度（对应[-180°, 180°]度），保证最短旋转路径
        rotateRad = normalizeRadian(rotateRad);

        // 8. 弧度转换为度，作为最终输出
        double deltaTheta = Math.toDegrees(rotateRad);

        // 保留6位小数（工程实际足够，避免精度冗余）
        return Math.round(deltaTheta * 1000000.0) / 1000000.0;
    }

    // ---------------------- 工具方法：弧度归一化到[-π, π] ----------------------
    /**
     * 将任意弧度值归一化到[-π, π]范围，对应角度[-180°, 180°]
     * @param rad 原始弧度值
     * @return 归一化后的弧度值∈[-π, π]
     */
    private double normalizeRadian(double rad) {
        // 先将弧度转换为[0, 2π)范围
        rad = rad % (2 * Math.PI);
        // 再转换为[-π, π)范围
        if (rad > Math.PI) {
            rad -= 2 * Math.PI;
        } else if (rad < -Math.PI) {
            rad += 2 * Math.PI;
        }
        return rad;
    }

    // ---------------------- 辅助getter方法（可选，用于外部获取目标位置/偏移参数） ----------------------
    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getTurretOffsetX() {
        return turretOffsetX;
    }

    public double getTurretOffsetY() {
        return turretOffsetY;
    }

    /**
    public static void main(String[] args) {
        // 1. 初始化计算器：炮台相对机器人中心偏移(1,0)（x轴正方向1个单位，y轴0）
        TurretAimingCalculator calculator = new TurretAimingCalculator(1.0, 0.0);

        // 2. 设置目标位置：世界坐标系(10, 5)
        calculator.setTargetPosition(10.0, 5.0);

        // 3. 机器人当前状态：
        double robotX = 0.0;    // 机器人中心在原点
        double robotY = 0.0;
        double theta = 0.0;     // 机器人方向沿x轴正方向（0度）
        double thetaTurret = 0.0;// 炮台当前相对机器人朝x轴正方向（0度）

        // 4. 计算旋转角
        double deltaTheta = calculator.calculateTurretRotateAngle(robotX, robotY, theta, thetaTurret);

        // 5. 输出结果
        System.out.println("炮台需要旋转的角度：" + deltaTheta + " 度");
        // 预期结果：约26.565051度（顺时针旋转26.57度，指向目标(10,5)）
     **/

}
