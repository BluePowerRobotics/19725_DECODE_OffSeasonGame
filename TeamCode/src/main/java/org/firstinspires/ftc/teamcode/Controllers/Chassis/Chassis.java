package org.firstinspires.ftc.teamcode.Controllers.Chassis;

public class Chassis {
    private ChassisController controller;
    double WanderSpeed;
    public Chassis(ChassisController controller,double WanderSpeed) {
        this.controller = controller;
        this.WanderSpeed = WanderSpeed;
    }

    public void stop(){
        controller.solveChassis(0, 0, 0);
    }
    public void GoTo(double targetTheta){ //targetTheta表示目标与小车正前方（x轴正方向）的夹角（逆时针为正）
        //前进的同时转向
        double vx = Math.cos(targetTheta) * WanderSpeed;
        double vy = Math.sin(targetTheta) * WanderSpeed;
        double k = ChassisController.PARAMS.maxOmega / (Math.PI / 2);
        double omega = targetTheta * k;
        controller.solveChassis(-vy, vx, omega); //坐标系变来变去好玩吗？
        //我真tm服了，Controller内部和其他程序一样都用x轴向前了，为什么要吃y轴向前的参数？
    }
    public void GoToShootingArea(){
        //todo: 实现Chassis移动到射击区域
    }
    public void SlowTurning(double omega){
        controller.solveChassis(0, 0, omega);
    }
    public void update(){
        //todo: 实现Chassis手动模式更新
    }
}
