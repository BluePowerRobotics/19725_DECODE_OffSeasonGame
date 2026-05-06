package org.firstinspires.ftc.teamcode.utility.RK4;

import org.firstinspires.ftc.teamcode.utility.HypParams;

public class ProjectileParameters {
    public double v0;
    public double kx;
    public double ky;
    public double m;
    public double g;
    public double deltaH;
    public double thetaMax;
    public double thetaMin;

    public ProjectileParameters() {
        this.v0 = 10.0;
        this.kx = 0.015;
        this.ky = 0.0;
        this.m = HypParams.ballMass;
        this.g = 9.81;
        this.deltaH = 0.5;
        this.thetaMin = Math.toRadians(45);
        this.thetaMax = Math.toRadians(65);
    }

    public ProjectileParameters(double v0, double kx, double ky, double m, double deltaH, double thetaMax) {
        this.v0 = v0;
        this.kx = kx;
        this.ky = ky;
        this.m = m;
        this.g = 9.81;
        this.deltaH = deltaH;
        this.thetaMin = Math.toRadians(45);
        this.thetaMax = thetaMax;
    }

    public ProjectileParameters withV0(double v0) {
        this.v0 = v0;
        return this;
    }

    public ProjectileParameters withKx(double kx) {
        this.kx = kx;
        return this;
    }

    public ProjectileParameters withKy(double ky) {
        this.ky = ky;
        return this;
    }

    public ProjectileParameters withM(double m) {
        this.m = m;
        return this;
    }

    public ProjectileParameters withDeltaH(double deltaH) {
        this.deltaH = deltaH;
        return this;
    }

    public ProjectileParameters withThetaMax(double thetaMax) {
        this.thetaMax = thetaMax;
        return this;
    }

    public ProjectileParameters copy() {
        ProjectileParameters copy = new ProjectileParameters(v0, kx, ky, m, deltaH, thetaMax);
        copy.thetaMin = this.thetaMin;
        return copy;
    }
}