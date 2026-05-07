package org.firstinspires.ftc.teamcode.Controllers.Limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Controllers.Limelight.projection.Projector;
import org.firstinspires.ftc.teamcode.utility.HypParams;

import java.util.*;

@Config
public class Tracker {
    private Detector detector;
    private Projector projector = new Projector(HypParams.Limelight_h, HypParams.Limelight_m0);
    private List<Target> targets;
    private List<CandidateTarget> candidateTargets;
    private int nextTargetId;
    private Map<Target, Integer> removalPending;
    public static double distanceThreshold = HypParams.distanceThreshold;
    public static int confirmationFrames = HypParams.confirmationFrames;
    public static int removalFrames = HypParams.removalFrames;

    public Tracker(HardwareMap hardwareMap) {
        this.detector = new Detector(hardwareMap);
        if (detector == null) {
            throw new RuntimeException("Detector initialization failed");
        }
        this.targets = new ArrayList<>();
        this.candidateTargets = new ArrayList<>();
        this.nextTargetId = 0;
        this.removalPending = new HashMap<>();
    }

    /**
     * 启动追踪器
     * 启动视觉检测器
     */
    public void start() {
        detector.start();
    }

    /**
     * 更新追踪状态
     * 1. 获取当前检测结果
     * 2. 直接处理每个检测（不再聚类，每个球单独视为一个目标）
     * 3. 匹配并更新已确认的目标
     * 4. 处理新的候选目标
     * 5. 处理缺失的目标
     */
    public void update() {
        List<Detection> currentDetections = getCurrentDetections();
        matchAndUpdateTargets(currentDetections);
        handleNewCandidates(currentDetections);
        handleMissingTargets();
    }

    /**
     * 获取当前检测结果
     * 从检测器获取紫色和绿色目标的中心点，转换为世界坐标后返回
     * @return 当前检测到的所有目标
     */
    private List<Detection> getCurrentDetections() {
        List<Detection> detections = new ArrayList<>();

        double[][] purpleOffsets = detector.get_center("purple");
        for (double[] offset : purpleOffsets) {
            double[] worldPos = projector.project(offset[0], offset[1]);
            detections.add(new Detection("purple", worldPos[0], worldPos[1]));
        }

        double[][] greenOffsets = detector.get_center("green");
        for (double[] offset : greenOffsets) {
            double[] worldPos = projector.project(offset[0], offset[1]);
            detections.add(new Detection("green", worldPos[0], worldPos[1]));
        }

        return detections;
    }

    /**
     * 聚类方法（已注释，不再使用）
     * 原方法使用广度优先搜索(BFS)算法，将距离小于等于 distanceThreshold 的检测归为一组
     */
    /*
    private List<List<Detection>> clusterDetections(List<Detection> detections) {
        List<List<Detection>> groups = new ArrayList<>();
        Set<Detection> processed = new HashSet<>();

        for (Detection detection : detections) {
            if (processed.contains(detection)) {
                continue;
            }

            List<Detection> group = new ArrayList<>();
            Queue<Detection> queue = new LinkedList<>();

            group.add(detection);
            queue.add(detection);
            processed.add(detection);

            while (!queue.isEmpty()) {
                Detection current = queue.poll();
                for (Detection other : detections) {
                    if (!processed.contains(other) && current.distanceTo(other) <= distanceThreshold) {
                        group.add(other);
                        queue.add(other);
                        processed.add(other);
                    }
                }
            }

            groups.add(group);
        }

        return groups;
    }
    */

    /**
     * 匹配并更新已确认的目标
     * 每个检测单独处理，不再使用聚类
     * @param currentDetections 当前的检测结果列表
     */
    private void matchAndUpdateTargets(List<Detection> currentDetections) {
        Set<Target> matchedTargets = new HashSet<>();

        for (Detection detection : currentDetections) {
            Target matchedTarget = null;
            double minDistance = Double.MAX_VALUE;

            for (Target target : targets) {
                if (matchedTargets.contains(target)) continue;
                // 匹配颜色相同且距离最近的目标
                if (!target.color.equals(detection.color)) continue;

                double distance = Math.sqrt(Math.pow(target.centerX - detection.x, 2) + Math.pow(target.centerY - detection.y, 2));
                if (distance <= distanceThreshold && distance < minDistance) {
                    matchedTarget = target;
                    minDistance = distance;
                }
            }

            if (matchedTarget != null) {
                matchedTarget.updateDetection(detection);
                matchedTarget.lastSeenTimestamp = System.currentTimeMillis();
                matchedTargets.add(matchedTarget);
                removalPending.remove(matchedTarget);
            }
        }

        for (Target target : targets) {
            if (!matchedTargets.contains(target)) {
                target.consecutiveMisses++;
                target.consecutiveFrames = 0;
                int count = removalPending.getOrDefault(target, 0) + 1;
                removalPending.put(target, count);
            }
        }
    }

    /**
     * 处理新的候选目标
     * 每个检测单独处理，不再使用聚类
     * @param currentDetections 当前的检测结果列表
     */
    private void handleNewCandidates(List<Detection> currentDetections) {
        Set<CandidateTarget> matchedCandidates = new HashSet<>();
        List<CandidateTarget> toRemove = new ArrayList<>();

        for (Detection detection : currentDetections) {
            CandidateTarget matchedCandidate = null;
            double minDistance = Double.MAX_VALUE;

            for (CandidateTarget candidate : candidateTargets) {
                // 匹配颜色相同且距离最近的候选目标
                if (!candidate.color.equals(detection.color)) continue;
                double distance = Math.hypot(candidate.centerX - detection.x, candidate.centerY - detection.y);
                if (distance <= distanceThreshold && distance < minDistance) {
                    matchedCandidate = candidate;
                    minDistance = distance;
                }
            }

            if (matchedCandidate != null) {
                matchedCandidate.updateDetection(detection);
                matchedCandidate.consecutiveFrames++;
                matchedCandidate.consecutiveMisses = 0;
                matchedCandidates.add(matchedCandidate);

                if (matchedCandidate.consecutiveFrames >= Tracker.confirmationFrames) {
                    Target newTarget = new Target(nextTargetId++, detection.color);
                    newTarget.updateDetection(detection);
                    targets.add(newTarget);
                    toRemove.add(matchedCandidate);
                }
            } else {
                CandidateTarget newCandidate = new CandidateTarget(detection);
                candidateTargets.add(newCandidate);
            }
        }

        for (CandidateTarget candidate : candidateTargets) {
            if (!matchedCandidates.contains(candidate)) {
                candidate.consecutiveMisses++;
                candidate.consecutiveFrames = 0;
            }
        }

        List<CandidateTarget> toRemoveFinal = new ArrayList<>();
        for (CandidateTarget candidate : candidateTargets) {
            if (candidate.consecutiveMisses >= removalFrames) {
                toRemoveFinal.add(candidate);
            }
        }
        toRemoveFinal.addAll(toRemove);
        candidateTargets.removeAll(toRemoveFinal);
    }

    /**
     * 处理缺失的目标
     * 移除连续缺失达到 removalFrames 的目标
     */
    private void handleMissingTargets() {
        List<Target> toRemove = new ArrayList<>();

        for (Map.Entry<Target, Integer> entry : removalPending.entrySet()) {
            Target target = entry.getKey();
            int count = entry.getValue();
            if (count >= Tracker.removalFrames) {
                toRemove.add(target);
            }
        }

        for (Target target : toRemove) {
            targets.remove(target);
            removalPending.remove(target);
        }
    }

    /**
     * 获取最优目标
     * 计算每个目标的得分，返回得分最高的目标
     * 得分公式：score = consecutiveFrames / distanceToCamera
     * @return 得分最高的目标，若没有目标则返回 null
     */
    public Target getBestTarget() {
        if (targets.isEmpty()) {
            return null;
        }

        Target bestTarget = null;
        double bestScore = -1;

        for (Target target : targets) {
            double score = computeTargetScore(target);
            if (score > bestScore) {
                bestScore = score;
                bestTarget = target;
            }
        }

        return bestTarget;
    }

    /**
     * 获取特定颜色的最优目标
     * 计算指定颜色的每个目标的得分，返回得分最高的目标
     * 得分公式：score = consecutiveFrames / distanceToCamera
     * @param color 目标颜色（"purple" 或 "green"）
     * @return 得分最高的指定颜色目标，若没有目标则返回 null
     */
    public Target getBestTarget(String color) {
        if (targets.isEmpty()) {
            return null;
        }

        Target bestTarget = null;
        double bestScore = -1;

        for (Target target : targets) {
            if (!target.color.equals(color)) continue;
            double score = computeTargetScore(target);
            if (score > bestScore) {
                bestScore = score;
                bestTarget = target;
            }
        }

        return bestTarget;
    }

    /**
     * 计算目标得分
     * 得分公式：score = (连续出现帧数) / distanceToCamera
     * 使用连续出现帧数作为权重，保留低通滤波效果
     * @param target 目标对象
     * @return 目标得分
     */
    private double computeTargetScore(Target target) {
        int frameCount = target.consecutiveFrames;
        double distance = Math.sqrt(Math.pow(target.centerX, 2) + Math.pow(target.centerY, 2));
        if (distance == 0) {
            distance = 0.1;
        }
        return frameCount / distance;
    }

    /**
     * 获取已确认的目标列表
     * @return 已确认的目标列表
     */
    public List<Target> getTargets() {
        return targets;
    }

    /**
     * 获取候选目标列表
     * @return 候选目标列表
     */
    public List<CandidateTarget> getCandidateTargets() {
        return candidateTargets;
    }

    /**
     * 停止追踪器
     * 停止视觉检测器
     */
    public void stop() {
        detector.stop();
    }

    /**
     * 检测结果类，表示单个目标的检测信息
     */
    public static class Detection {
        public final String color;  // 目标颜色
        public final double x;      // 目标 x 坐标
        public final double y;      // 目标 y 坐标

        /**
         * 构造函数
         * @param color 目标颜色
         * @param x 目标 x 坐标
         * @param y 目标 y 坐标
         */
        public Detection(String color, double x, double y) {
            this.color = color;
            this.x = x;
            this.y = y;
        }

        /**
         * 计算与另一个检测结果的距离
         * @param other 另一个检测结果
         * @return 距离
         */
        public double distanceTo(Detection other) {
            return Math.sqrt(Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2));
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Detection that = (Detection) o;
            return Double.compare(that.x, x) == 0 && Double.compare(that.y, y) == 0 && Objects.equals(color, that.color);
        }

        @Override
        public int hashCode() {
            return Objects.hash(color, x, y);
        }
    }

    /**
     * 已确认的目标类，包含单个球的信息
     * 保留低通滤波功能（通过连续出现帧数实现）
     */
    public static class Target {
        public int id;                      // 目标 ID
        public String color;                // 目标颜色
        public double centerX;              // 目标中心点 x 坐标
        public double centerY;              // 目标中心点 y 坐标
        public long lastSeenTimestamp;      // 最后一次看到目标的时间戳
        public int consecutiveFrames;       // 连续出现帧数（用于低通滤波）
        public int consecutiveMisses;       // 连续缺失帧数

        /**
         * 构造函数
         * @param id 目标 ID
         * @param color 目标颜色
         */
        public Target(int id, String color) {
            this.id = id;
            this.color = color;
            this.centerX = 0;
            this.centerY = 0;
            this.lastSeenTimestamp = System.currentTimeMillis();
            this.consecutiveFrames = 0;
            this.consecutiveMisses = 0;
        }

        /**
         * 更新目标的检测信息
         * 使用指数移动平均进行低通滤波
         * @param detection 新的检测结果
         */
        public void updateDetection(Detection detection) {
            double alpha = 0.7; // 低通滤波系数
            this.centerX = alpha * detection.x + (1 - alpha) * this.centerX;
            this.centerY = alpha * detection.y + (1 - alpha) * this.centerY;
            this.consecutiveFrames++;
            this.consecutiveMisses = 0;
        }

        /**
         * 获取目标到摄像头的距离
         * @return 距离
         */
        public double getDistanceToCamera() {
            return Math.sqrt(Math.pow(centerX, 2) + Math.pow(centerY, 2));
        }
    }

    /**
     * 候选目标类，尚未达到确认帧数的目标
     */
    public static class CandidateTarget {
        public String color;                // 目标颜色
        public double centerX;              // 候选目标中心点 x 坐标
        public double centerY;              // 候选目标中心点 y 坐标
        public int consecutiveFrames;       // 连续出现帧数
        public int consecutiveMisses;       // 连续缺失帧数

        /**
         * 构造函数
         * @param detection 检测结果
         */
        public CandidateTarget(Detection detection) {
            this.color = detection.color;
            this.centerX = detection.x;
            this.centerY = detection.y;
            this.consecutiveFrames = 1;
            this.consecutiveMisses = 0;
        }

        /**
         * 更新候选目标的检测信息
         * 使用指数移动平均进行低通滤波
         * @param detection 新的检测结果
         */
        public void updateDetection(Detection detection) {
            double alpha = 0.7; // 低通滤波系数
            this.centerX = alpha * detection.x + (1 - alpha) * this.centerX;
            this.centerY = alpha * detection.y + (1 - alpha) * this.centerY;
        }
    }
}