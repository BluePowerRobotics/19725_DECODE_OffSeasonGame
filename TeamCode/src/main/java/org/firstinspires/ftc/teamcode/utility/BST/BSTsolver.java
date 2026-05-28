package org.firstinspires.ftc.teamcode.utility.BST;

import org.firstinspires.ftc.teamcode.utility.LinearInterpolation.LinearInterpolator;

import java.util.List;

public class BSTsolver {
    //单位：英寸
    public static class Solution {
        public final double roll;
        public final int yaw;
        public final int speed;
        public final boolean success;
        public final String message;

        public Solution(double roll, int yaw, int speed) {
            this.roll = roll;
            this.yaw = yaw;
            this.speed = speed;
            this.success = true;
            this.message = "Success";
        }

        public Solution(String message) {
            this.roll = 0;
            this.yaw = 0;
            this.speed = 0;
            this.success = false;
            this.message = message;
        }
    }

    private LinearInterpolator interpolator;
    private static final double EPSILON = 0.001;
    private static final int MAX_ITERATIONS = 50;

    public BSTsolver() {
        this.interpolator = new LinearInterpolator();
    }

    public BSTsolver(LinearInterpolator interpolator) {
        this.interpolator = interpolator;
    }

    public Solution predict(double vx, double vy, double dx, double dy) {
        List<Integer> yaws = interpolator.getAvailableYaws();
        if (yaws.isEmpty()) {
            return new Solution("No yaw data available");
        }

        int bestYaw = -1;
        int bestSpeed = 0;
        double bestRoll = 0;
        double bestResidual = Double.MAX_VALUE;

        for (int yaw : yaws) {
            Double solvedTime = solveForTime(vx, vy, dx, dy, yaw);
            if (solvedTime == null) {
                continue;
            }

            Double distance = interpolator.getDistanceByTime(yaw, solvedTime);
            Integer speed = interpolator.getSpeedByDistance(yaw, distance);

            if (speed == null) {
                continue;
            }

            double compensatedDx = dx - vx * solvedTime;
            double compensatedDy = dy - vy * solvedTime;
            double roll = Math.atan2(compensatedDy, compensatedDx);

            double residual = Math.abs(computeF(solvedTime, vx, vy, dx, dy, yaw));

            if (residual < bestResidual) {
                bestResidual = residual;
                bestYaw = yaw;
                bestSpeed = speed;
                bestRoll = roll;
            }
        }

        if (bestYaw == -1) {
            return new Solution("No valid solution found for any yaw");
        }

        return new Solution(bestRoll, bestYaw, bestSpeed);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    private Double solveForTime(double vx, double vy, double dx, double dy, int yaw) {
        double tMin = 0.0;
        double tMax = interpolator.getMaxTime(yaw);

        if (tMax <= 0) {
            return null;
        }

        double speedSquared = vx * vx + vy * vy;
        double dotProduct = dx * vx + dy * vy;

        if (speedSquared > 0 && dotProduct > 0) {
            double tCritical = dotProduct / speedSquared;
            tMax = Math.min(tMax, tCritical);
        }

        Double fMin = computeF(tMin, vx, vy, dx, dy, yaw);
        Double fMax = computeF(tMax, vx, vy, dx, dy, yaw);

        if (fMin == null || fMax == null) {
            return null;
        }

        if (fMin * fMax > 0) {
            return null;
        }

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double tMid = (tMin + tMax) / 2;
            Double fMid = computeF(tMid, vx, vy, dx, dy, yaw);

            if (fMid == null) {
                return null;
            }

            if (Math.abs(fMid) < EPSILON) {
                return tMid;
            }

            if (fMin * fMid < 0) {
                tMax = tMid;
                fMax = fMid;
            } else {
                tMin = tMid;
                fMin = fMid;
            }
        }

        return (tMin + tMax) / 2;
    }

    private Double computeF(double t, double vx, double vy, double dx, double dy, int yaw) {
        if (t <= 0.0) {
            return Math.hypot(dx, dy);
        }

        double compensatedDx = dx - vx * t;
        double compensatedDy = dy - vy * t;
        double L = Math.hypot(compensatedDx, compensatedDy);

        Double d = interpolator.getDistanceByTime(yaw, t);
        if (d == null) {
            return null;
        }

        return L - d;
    }
}