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
    private static int nextMemberId;
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
        nextMemberId = 0;
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
     * 2. 对检测结果进行聚类
     * 3. 匹配并更新已确认的目标
     * 4. 处理新的候选目标
     * 5. 处理缺失的目标
     */
    public void update() {
        List<Detection> currentDetections = getCurrentDetections();
        List<List<Detection>> currentGroups = clusterDetections(currentDetections);
        matchAndUpdateTargets(currentGroups);
        handleNewCandidates(currentGroups);
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
     * 对检测结果进行聚类
     * 使用广度优先搜索(BFS)算法，将距离小于等于 distanceThreshold 的检测归为一组
     * 单个孤立的检测也会形成大小为 1 的组
     * @param detections 待聚类的检测结果
     * @return 聚类后的检测组列表
     */
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

    /**
     * 匹配并更新已确认的目标
     * 1. 对每个检测组计算中心点
     * 2. 寻找距离最近的已确认目标
     * 3. 更新匹配到的目标
     * 4. 标记未匹配到的目标为缺失
     * 5. 移除目标中过期的成员
     * @param currentGroups 当前的检测组列表
     */
    private void matchAndUpdateTargets(List<List<Detection>> currentGroups) {
        Set<Target> matchedTargets = new HashSet<>();

        for (List<Detection> group : currentGroups) {
            double centerX = 0, centerY = 0;
            for (Detection detection : group) {
                centerX += detection.x;
                centerY += detection.y;
            }
            centerX /= group.size();
            centerY /= group.size();

            Target matchedTarget = null;
            double minDistance = Double.MAX_VALUE;

            for (Target target : targets) {
                if (matchedTargets.contains(target)) continue;

                double distance = Math.sqrt(Math.pow(target.centerX - centerX, 2) + Math.pow(target.centerY - centerY, 2));
                if (distance <= distanceThreshold && distance < minDistance) {
                    matchedTarget = target;
                    minDistance = distance;
                }
            }

                if (matchedTarget != null) {
                matchedTarget.updateGroupAndMembers(group, Tracker.confirmationFrames);
                matchedTarget.lastSeenTimestamp = System.currentTimeMillis();
                matchedTargets.add(matchedTarget);
                removalPending.remove(matchedTarget);
            }
        }

        for (Target target : targets) {
            if (!matchedTargets.contains(target)) {
                target.markAllMembersMissed();
                int count = removalPending.getOrDefault(target, 0) + 1;
                removalPending.put(target, count);
            }
        }

        List<Target> toCheckEmpty = new ArrayList<>(targets);
        for (Target target : toCheckEmpty) {
            target.removeStaleMembers(removalFrames, confirmationFrames);
            if (target.isEmpty()) {
                removalPending.put(target, removalFrames);
            }
        }
    }

    /**
     * 处理新的候选目标
     * 1. 对每个检测组计算中心点
     * 2. 寻找距离最近的候选目标
     * 3. 更新匹配到的候选目标
     * 4. 对连续出现达到 confirmationFrames 的候选目标，将其转换为已确认目标
     * 5. 对未匹配到的候选目标，增加其缺失计数
     * 6. 移除连续缺失达到 removalFrames 的候选目标
     * @param currentGroups 当前的检测组列表
     */
    private void handleNewCandidates(List<List<Detection>> currentGroups) {
        Set<CandidateTarget> matchedCandidates = new HashSet<>();
        List<CandidateTarget> toRemove = new ArrayList<>();

        for (List<Detection> group : currentGroups) {
            double centerX = 0, centerY = 0;
            for (Detection detection : group) {
                centerX += detection.x;
                centerY += detection.y;
            }
            centerX /= group.size();
            centerY /= group.size();

            CandidateTarget matchedCandidate = null;
            double minDistance = Double.MAX_VALUE;

            for (CandidateTarget candidate : candidateTargets) {
                double distance = Math.hypot(candidate.centerX - centerX, candidate.centerY - centerY);
                if (distance <= distanceThreshold && distance < minDistance) {
                    matchedCandidate = candidate;
                    minDistance = distance;
                }
            }

            if (matchedCandidate != null) {
                matchedCandidate.updateGroupAndMembers(group);
                matchedCandidate.consecutiveFrames++;
                matchedCandidate.consecutiveMisses = 0;
                matchedCandidates.add(matchedCandidate);

                if (matchedCandidate.consecutiveFrames >= Tracker.confirmationFrames) {
                    Target newTarget = new Target(nextTargetId++);
                    newTarget.updateGroupAndMembers(matchedCandidate.members, Tracker.confirmationFrames);
                    targets.add(newTarget);
                    toRemove.add(matchedCandidate);
                }
            } else {
                CandidateTarget newCandidate = new CandidateTarget(nextMemberId++, group);
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
     * 移除连续缺失达到 removalFrames 且没有活跃成员的目标
     */
    private void handleMissingTargets() {
        List<Target> toRemove = new ArrayList<>();

        for (Map.Entry<Target, Integer> entry : removalPending.entrySet()) {
            Target target = entry.getKey();
            int count = entry.getValue();
            if (count >= Tracker.removalFrames && target.getActiveMemberCount(Tracker.confirmationFrames) == 0) {
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
     * 得分公式：score = activeMemberCount / distanceToCamera
     * @return 得分最高的目标，若没有目标则返回 null
     */
    public Target getBestTarget() {
        if (targets.isEmpty()) {
            return null;
        }

        Target bestTarget = null;
        double bestScore = -1;

        for (Target target : targets) {
            if (target.isEmpty()) continue;
            if (target.getActiveMemberCount(Tracker.confirmationFrames) == 0) continue;
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
     * 得分公式：score = activeMemberCount / distanceToCamera
     * @param target 目标对象
     * @return 目标得分
     */
    private double computeTargetScore(Target target) {
        int memberCount = target.getActiveMemberCount(Tracker.confirmationFrames);
        double distance = Math.sqrt(Math.pow(target.centerX, 2) + Math.pow(target.centerY, 2));
        if (distance == 0) {
            distance = 0.1;
        }
        return memberCount / distance;
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
     * 已确认的目标类，包含多个成员并维护目标的中心点坐标
     */
    public static class Target {
        public int id;                      // 目标 ID
        public Map<Integer, Member> members;  // 目标的成员列表，key 为成员 ID
        public double centerX;              // 目标中心点 x 坐标
        public double centerY;              // 目标中心点 y 坐标
        public long lastSeenTimestamp;      // 最后一次看到目标的时间戳

        /**
         * 构造函数
         * @param id 目标 ID
         */
        public Target(int id) {
            this.id = id;
            this.members = new HashMap<>();
            this.centerX = 0;
            this.centerY = 0;
            this.lastSeenTimestamp = System.currentTimeMillis();
        }

        /**
         * 更新目标的成员和中心点
         * @param group 检测组
         * @param confirmationFrames 确认帧数
         */
        public void updateGroupAndMembers(List<Detection> group, int confirmationFrames) {
            Set<Integer> processedMemberIds = new HashSet<>();

            for (Detection detection : group) {
                Member matchedMember = null;
                double minDistance = Double.MAX_VALUE;

                for (Map.Entry<Integer, Member> entry : members.entrySet()) {
                    Member member = entry.getValue();
                    if (member.detection.color.equals(detection.color)) {
                        double distance = member.detection.distanceTo(detection);
                        if (distance < minDistance) {
                            minDistance = distance;
                            matchedMember = member;
                        }
                    }
                }

                if (matchedMember != null && minDistance <= 0.3) {
                    matchedMember.consecutiveFrames++;
                    matchedMember.consecutiveMisses = 0;
                    matchedMember.detection = detection;
                    processedMemberIds.add(matchedMember.id);
                } else {
                    Member newMember = new Member(Tracker.nextMemberId++, detection);
                    members.put(newMember.id, newMember);
                    processedMemberIds.add(newMember.id);
                }
            }

            for (Map.Entry<Integer, Member> entry : members.entrySet()) {
                if (!processedMemberIds.contains(entry.getKey())) {
                    Member m = entry.getValue();
                    m.consecutiveMisses++;
                    m.consecutiveFrames = 0;
                }
            }

            updateCenter(confirmationFrames);
        }

        /**
         * 使用现有成员更新目标
         * @param existingMembers 现有成员
         * @param confirmationFrames 确认帧数
         */
        public void updateGroupAndMembers(Map<Integer, Member> existingMembers, int confirmationFrames) {
            this.members = new HashMap<>(existingMembers);
            updateCenter(confirmationFrames);
        }

        /**
         * 标记所有成员为缺失
         */
        public void markAllMembersMissed() {
            for (Member member : members.values()) {
                member.consecutiveMisses++;
                member.consecutiveFrames = 0;
            }
        }

        /**
         * 移除过期的成员
         * @param removalFrames 移除所需的连续缺失帧数
         * @param confirmationFrames 确认帧数
         */
        public void removeStaleMembers(int removalFrames, int confirmationFrames) {
            List<Integer> toRemove = new ArrayList<>();
            for (Map.Entry<Integer, Member> entry : members.entrySet()) {
                if (entry.getValue().consecutiveMisses >= removalFrames) {
                    toRemove.add(entry.getKey());
                }
            }
            for (Integer id : toRemove) {
                members.remove(id);
            }
            updateCenter(confirmationFrames);
        }

        /**
         * 获取活跃成员数量
         * @param confirmationFrames 确认帧数
         * @return 活跃成员数量
         */
        public int getActiveMemberCount(int confirmationFrames) {
            int count = 0;
            for (Member member : members.values()) {
                if (member.consecutiveFrames >= confirmationFrames) {
                    count++;
                }
            }
            return count;
        }

        /**
         * 更新目标中心点
         * 仅使用连续出现帧数 >= confirmationFrames 的成员坐标计算平均值
         * @param confirmationFrames 确认帧数
         */
        public void updateCenter(int confirmationFrames) {
            int activeCount = 0;
            double sumX = 0, sumY = 0;
            for (Member member : members.values()) {
                if (member.consecutiveFrames >= confirmationFrames) {
                    sumX += member.detection.x;
                    sumY += member.detection.y;
                    activeCount++;
                }
            }

            if (activeCount == 0) {
                centerX = 0;
                centerY = 0;
            } else {
                centerX = sumX / activeCount;
                centerY = sumY / activeCount;
            }
        }

        /**
         * 检查目标是否为空
         * @return 是否为空
         */
        public boolean isEmpty() {
            return members.isEmpty();
        }

        /**
         * 获取目标到摄像头的距离
         * @return 距离
         */
        public double getDistanceToCamera() {
            return Math.sqrt(Math.pow(centerX, 2) + Math.pow(centerY, 2));
        }

        /**
         * 目标成员类，表示目标中的单个检测结果
         */
        public static class Member {
            public int id;                 // 成员 ID
            public Detection detection;    // 检测结果
            public int consecutiveFrames;  // 连续出现帧数
            public int consecutiveMisses;  // 连续缺失帧数

            /**
             * 构造函数
             * @param id 成员 ID
             * @param detection 检测结果
             */
            public Member(int id, Detection detection) {
                this.id = id;
                this.detection = detection;
                this.consecutiveFrames = 1;
                this.consecutiveMisses = 0;
            }
        }
    }

    /**
     * 候选目标类，尚未达到确认帧数的目标
     */
    public static class CandidateTarget {
        public Map<Integer, Target.Member> members;  // 候选目标的成员列表
        public double centerX;                      // 候选目标中心点 x 坐标
        public double centerY;                      // 候选目标中心点 y 坐标
        public int consecutiveFrames;              // 连续出现帧数
        public int consecutiveMisses;              // 连续缺失帧数
        private int nextMemberId;                  // 下一个成员 ID

        /**
         * 构造函数
         * @param startMemberId 起始成员 ID
         * @param group 检测组
         */
        public CandidateTarget(int startMemberId, List<Detection> group) {
            this.members = new HashMap<>();
            this.nextMemberId = startMemberId;
            for (Detection detection : group) {
                Target.Member member = new Target.Member(Tracker.nextMemberId++, detection);
                members.put(member.id, member);
            }
            this.consecutiveFrames = 1;
            this.consecutiveMisses = 0;
            updateCenter();
        }

        /**
         * 更新候选目标的成员和中心点
         * @param group 检测组
         */
        public void updateGroupAndMembers(List<Detection> group) {
            Set<Integer> processedMemberIds = new HashSet<>();

            for (Detection detection : group) {
                Target.Member matchedMember = null;
                double minDistance = Double.MAX_VALUE;

                for (Map.Entry<Integer, Target.Member> entry : members.entrySet()) {
                    Target.Member member = entry.getValue();
                    if (member.detection.color.equals(detection.color)) {
                        double distance = member.detection.distanceTo(detection);
                        if (distance < minDistance) {
                            minDistance = distance;
                            matchedMember = member;
                        }
                    }
                }

                if (matchedMember != null && minDistance <= 0.3) {
                    matchedMember.consecutiveFrames++;
                    matchedMember.consecutiveMisses = 0;
                    matchedMember.detection = detection;
                    processedMemberIds.add(matchedMember.id);
                } else {
                    Target.Member newMember = new Target.Member(Tracker.nextMemberId++, detection);
                    members.put(newMember.id, newMember);
                    processedMemberIds.add(newMember.id);
                }
            }

            for (Map.Entry<Integer, Target.Member> entry : members.entrySet()) {
                if (!processedMemberIds.contains(entry.getKey())) {
                    Target.Member m = entry.getValue();
                    m.consecutiveMisses++;
                    m.consecutiveFrames = 0;
                }
            }

            updateCenter();
        }

        /**
         * 更新候选目标中心点
         * 使用所有成员的坐标计算平均值
         */
        private void updateCenter() {
            if (members.isEmpty()) {
                centerX = 0;
                centerY = 0;
                return;
            }

            double sumX = 0, sumY = 0;
            for (Target.Member member : members.values()) {
                sumX += member.detection.x;
                sumY += member.detection.y;
            }

            centerX = sumX / members.size();
            centerY = sumY / members.size();
        }
    }
}