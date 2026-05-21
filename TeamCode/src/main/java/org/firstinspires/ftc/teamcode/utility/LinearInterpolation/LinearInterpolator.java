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

    private static class DataPoint {
        double distance;
        int speed;

        DataPoint(double distance, int speed) {
            this.distance = distance;
            this.speed = speed;
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
                if (parts.length >= 3) {
                    int yaw = Integer.parseInt(parts[0].trim());
                    int speed = Integer.parseInt(parts[1].trim());
                    double distance = Double.parseDouble(parts[2].trim());
                    
                    yawDataMap.computeIfAbsent(yaw, k -> new ArrayList<>()).add(new DataPoint(distance, speed));
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

            int insertionIndex = Collections.binarySearch(dataPoints, new DataPoint(targetDistance, 0), 
                Comparator.comparingDouble(p -> p.distance));
            
            if (insertionIndex < 0) {
                insertionIndex = -insertionIndex - 1;
            }

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
}