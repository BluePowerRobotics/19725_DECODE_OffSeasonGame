package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Detector {
    private Limelight3A limelight;
    private static final int PIPELINE_INDEX = 5; // 使用pipeline5

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     */
    public Detector(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }

    public void start(){
        if (limelight != null) {
            limelight.start();
        }
    }
    /**
     * 获取指定对象的归一化坐标
     * @param objectName 对象名称
     * @return 二维数组，每行表示一个对象的归一化坐标 [m, n]，m为x方向偏移(左正右负)，n为y方向偏移(上正下负)
     */
    public double[][] get_center(String objectName) {
        LLResult result = limelight.getLatestResult();
        List<double[]> centers = new ArrayList<>();

        if (result != null && result.isValid()) {
            // 获取检测器结果
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            for (LLResultTypes.DetectorResult dr : detectorResults) {
                // 检查对象名称是否匹配（包括带?的情况）
                String className = dr.getClassName();
                if (className.equals(objectName) || className.equals(objectName + "\r")) {
                    // 获取像素坐标偏移（相对于画面中心）
                    double txp = dr.getTargetXPixels();
                    double typ = dr.getTargetYPixels();
                  
                    // 归一化到0-1范围（取画面长边为1，假设图像分辨率为640x480）
                    // m对应x方向偏移，n对应y方向偏移
                    double m = (320-txp) / 320.0; // x方向归一化，左侧为正
                    double n = (240-typ) / 320.0; // y方向归一化，上方为正（均取长边320为基准）

                    centers.add(new double[]{m, n});
                }
            }
        }

        // 将List转换为二维数组
        double[][] resultArray = new double[centers.size()][2];
        for (int i = 0; i < centers.size(); i++) {
            resultArray[i] = centers.get(i);
        }

        return resultArray;
    }

    /**
     * 停止Limelight
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
    
    /**
     * 输出所有检测对象的全部信息
     * @return 所有检测对象的详细信息，每行一个对象
     */
    public boolean hasTarget(){
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }
    public double bearing(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //逆时针为正
            return -result.getTx(); //可能有问题，记得实验
        }
        return 0.0;
    }

    /**
     * 获取当前检测到的目标总数
     * @return 检测到的目标数量，无效结果返回0
     */
    public int targetNum() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            return detectorResults != null ? detectorResults.size() : 0;
        }
        return 0;
    }

    public List<String> PrintAll() {
        List<String> allInfo = new ArrayList<>();
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid()) {
            // 获取检测器结果
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            
            for (int i = 0; i < detectorResults.size(); i++) {
                LLResultTypes.DetectorResult dr = detectorResults.get(i);
                StringBuilder info = new StringBuilder();
                info.append("Object " + i + ": ");
                info.append("Name=" + dr.getClassName() + ", ");
                info.append("X=" + dr.getTargetXPixels() + ", ");
                info.append("Y=" + dr.getTargetYPixels() + ", ");
                info.append("Confidence=" + dr.getConfidence() + ", ");
                info.append("Area=" + dr.getTargetArea());
                allInfo.add(info.toString());
            }
        } else {
            allInfo.add("No valid Limelight results");
        }
        
        return allInfo;
    }
}