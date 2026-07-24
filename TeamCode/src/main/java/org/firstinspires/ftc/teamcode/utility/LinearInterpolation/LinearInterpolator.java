package org.firstinspires.ftc.teamcode.utility.LinearInterpolation;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LinearInterpolator {

    public static class DataPoint {
        public final double distance;
        public final int speed;
        public final double t;

        public DataPoint(double distance, int speed, double t) {
            this.distance = distance;
            this.speed = speed;
            this.t = t;
        }
    }

    public static class PredictionResult {
        public final int yaw;
        public final int speed;

        public PredictionResult(int yaw, int speed) {
            this.yaw = yaw;
            this.speed = speed;
        }
    }

    private Map<Integer, List<DataPoint>> yawDataMap;

    public LinearInterpolator() {
        yawDataMap = new HashMap<>();
        loadDataFromCSV();
    }

    /**
     * 从指定资源路径加载无时间列的 CSV 数据（格式：yaw,speed,distance）
     * @param resourcePath 资源文件名（相对于本类所在目录）
     */
    public LinearInterpolator(String resourcePath) {
        yawDataMap = new HashMap<>();
        loadDataFromCSVWithoutTime(resourcePath);
    }

    /**
     * 加载无时间列的 CSV 数据（格式：yaw,speed,distance）
     */
    private void loadDataFromCSVWithoutTime(String resourcePath) {
        try {
            InputStream inputStream = getClass().getResourceAsStream(resourcePath);
            if (inputStream == null) {
                return;
            }

            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            String line;
            boolean isFirstLine = true;

            while ((line = reader.readLine()) != null) {
                if (isFirstLine) {
                    isFirstLine = false;
                    continue;
                }

                String[] parts = line.split(",");
                if (parts.length >= 3) {
                    int yaw = Integer.parseInt(parts[0].trim());
                    int speed = Integer.parseInt(parts[1].trim());
                    double distance = Double.parseDouble(parts[2].trim());

                    yawDataMap.computeIfAbsent(yaw, k -> new ArrayList<>()).add(new DataPoint(distance, speed, 0));
                }
            }

            for (List<DataPoint> dataPoints : yawDataMap.values()) {
                dataPoints.sort(Comparator.comparingDouble(p -> p.distance));
            }

            reader.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void loadDataFromCSV() {
        try {
            InputStream inputStream = getClass().getResourceAsStream("data.csv");
            if (inputStream == null) {
                return;
            }

            BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream));
            String line;
            boolean isFirstLine = true;

            while ((line = reader.readLine()) != null) {
                if (isFirstLine) {
                    isFirstLine = false;
                    continue;
                }

                String[] parts = line.split(",");
                if (parts.length >= 4) {
                    int yaw = Integer.parseInt(parts[0].trim());
                    int speed = Integer.parseInt(parts[1].trim());
                    double distance = Double.parseDouble(parts[2].trim());
                    double t = Double.parseDouble(parts[3].trim());

                    yawDataMap.computeIfAbsent(yaw, k -> new ArrayList<>()).add(new DataPoint(distance, speed, t));
                }
            }

            for (List<DataPoint> dataPoints : yawDataMap.values()) {
                dataPoints.sort(Comparator.comparingDouble(p -> p.t));
                if (dataPoints.get(0).t > 1e-6) {
                    dataPoints.add(0, new DataPoint(0.0, 0, 0.0));
                }
            }

            reader.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public PredictionResult predict(double targetDistance) {
        if (yawDataMap.isEmpty()) {
            return null;
        }

        Integer bestYaw = null;
        Integer bestSpeed = null;
        int minSpeedDiff = Integer.MAX_VALUE;

        for (Map.Entry<Integer, List<DataPoint>> entry : yawDataMap.entrySet()) {
            int yaw = entry.getKey();
            List<DataPoint> dataPoints = entry.getValue();

            if (dataPoints.size() < 2) {
                continue;
            }

            int insertionIndex = Collections.binarySearch(dataPoints, new DataPoint(targetDistance, 0, 0),
                    Comparator.comparingDouble(p -> p.distance));

            if (insertionIndex >= 0) {
                bestYaw = yaw;
                bestSpeed = dataPoints.get(insertionIndex).speed;
                break;
            }

            insertionIndex = -insertionIndex - 1;

            if (insertionIndex <= 0 || insertionIndex >= dataPoints.size()) {
                continue;
            }

            DataPoint lowerPoint = dataPoints.get(insertionIndex - 1);
            DataPoint upperPoint = dataPoints.get(insertionIndex);

            int speedDiff = Math.abs(upperPoint.speed - lowerPoint.speed);

            if (speedDiff < minSpeedDiff) {
                minSpeedDiff = speedDiff;

                double t = (targetDistance - lowerPoint.distance) / (upperPoint.distance - lowerPoint.distance);
                int predictedSpeed = (int) Math.round(lowerPoint.speed + t * (upperPoint.speed - lowerPoint.speed));

                bestYaw = yaw;
                bestSpeed = predictedSpeed;
            }
        }

        if (bestYaw == null || bestSpeed == null) {
            return null;
        }

        return new PredictionResult(bestYaw, bestSpeed);
    }

    public Double getDistanceByTime(int yaw, double t) {
        List<DataPoint> dataPoints = yawDataMap.get(yaw);
        if (dataPoints == null || dataPoints.size() < 2) {
            return null;
        }

        dataPoints.sort(Comparator.comparingDouble(p -> p.t));

        DataPoint first = dataPoints.get(0);
        DataPoint last = dataPoints.get(dataPoints.size() - 1);

        if (Math.abs(t - first.t) < 1e-6) {
            return first.distance;
        }
        if (Math.abs(t - last.t) < 1e-6) {
            return last.distance;
        }
        if (t < first.t || t > last.t) {
            return null;
        }

        int idx = Collections.binarySearch(dataPoints, new DataPoint(0, 0, t),
                Comparator.comparingDouble(p -> p.t));

        if (idx >= 0) {
            return dataPoints.get(idx).distance;
        }

        idx = -idx - 1;

        if (idx == 0) {
            DataPoint lower = dataPoints.get(0);
            DataPoint upper = dataPoints.get(1);
            double ratio = (t - lower.t) / (upper.t - lower.t);
            return lower.distance + ratio * (upper.distance - lower.distance);
        }

        if (idx >= dataPoints.size()) {
            DataPoint lower = dataPoints.get(dataPoints.size() - 2);
            DataPoint upper = dataPoints.get(dataPoints.size() - 1);
            double ratio = (t - lower.t) / (upper.t - lower.t);
            return lower.distance + ratio * (upper.distance - lower.distance);
        }

        DataPoint lower = dataPoints.get(idx - 1);
        DataPoint upper = dataPoints.get(idx);
        double ratio = (t - lower.t) / (upper.t - lower.t);
        return lower.distance + ratio * (upper.distance - lower.distance);
    }

    public Integer getSpeedByDistance(int yaw, double distance) {
        List<DataPoint> dataPoints = yawDataMap.get(yaw);
        if (dataPoints == null || dataPoints.size() < 2) {
            return null;
        }

        dataPoints.sort(Comparator.comparingDouble(p -> p.distance));

        DataPoint first = dataPoints.get(0);
        DataPoint last = dataPoints.get(dataPoints.size() - 1);

        if (Math.abs(distance - first.distance) < 1e-6) {
            return first.speed;
        }
        if (Math.abs(distance - last.distance) < 1e-6) {
            return last.speed;
        }
        if (distance < first.distance || distance > last.distance) {
            return null;
        }

        int idx = Collections.binarySearch(dataPoints, new DataPoint(distance, 0, 0),
                Comparator.comparingDouble(p -> p.distance));

        if (idx >= 0) {
            return dataPoints.get(idx).speed;
        }

        idx = -idx - 1;

        if (idx == 0) {
            DataPoint lower = dataPoints.get(0);
            DataPoint upper = dataPoints.get(1);
            double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
            return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
        }

        if (idx >= dataPoints.size()) {
            DataPoint lower = dataPoints.get(dataPoints.size() - 2);
            DataPoint upper = dataPoints.get(dataPoints.size() - 1);
            double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
            return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
        }

        DataPoint lower = dataPoints.get(idx - 1);
        DataPoint upper = dataPoints.get(idx);
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
    }

    public List<Integer> getAvailableYaws() {
        return new ArrayList<>(yawDataMap.keySet());
    }

    /**
     * 获取所有可用仰角中的最小值
     * @return 最小 yaw 值，若无数据返回 null
     */
    public Integer getMinYaw() {
        if (yawDataMap.isEmpty()) return null;
        return yawDataMap.keySet().stream().min(Integer::compareTo).orElse(null);
    }

    /**
     * 对指定仰角，根据距离线性插值计算射速
     * @param yaw 仰角
     * @param distance 目标距离
     * @return 插值后的射速，超出范围返回 null
     */
    public Integer getSpeedForYaw(int yaw, double distance) {
        List<DataPoint> dataPoints = yawDataMap.get(yaw);
        if (dataPoints == null || dataPoints.size() < 2) {
            return null;
        }

        dataPoints.sort(Comparator.comparingDouble(p -> p.distance));

        DataPoint first = dataPoints.get(0);
        DataPoint last = dataPoints.get(dataPoints.size() - 1);

        if (distance < first.distance || distance > last.distance) {
            return null;
        }

        int idx = Collections.binarySearch(dataPoints, new DataPoint(distance, 0, 0),
                Comparator.comparingDouble(p -> p.distance));

        if (idx >= 0) {
            return dataPoints.get(idx).speed;
        }

        idx = -idx - 1;

        if (idx == 0) {
            DataPoint lower = dataPoints.get(0);
            DataPoint upper = dataPoints.get(1);
            double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
            return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
        }

        if (idx >= dataPoints.size()) {
            DataPoint lower = dataPoints.get(dataPoints.size() - 2);
            DataPoint upper = dataPoints.get(dataPoints.size() - 1);
            double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
            return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
        }

        DataPoint lower = dataPoints.get(idx - 1);
        DataPoint upper = dataPoints.get(idx);
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return (int) Math.round(lower.speed + ratio * (upper.speed - lower.speed));
    }

    public double getMaxTime(int yaw) {
        List<DataPoint> dataPoints = yawDataMap.get(yaw);
        if (dataPoints == null || dataPoints.isEmpty()) {
            return 0;
        }
        dataPoints.sort(Comparator.comparingDouble(p -> p.t));
        return dataPoints.get(dataPoints.size() - 1).t;
    }
}