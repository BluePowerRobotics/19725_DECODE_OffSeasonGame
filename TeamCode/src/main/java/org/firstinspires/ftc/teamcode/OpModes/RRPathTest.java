package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.lang.Math;

/**
 * RoadRunner 路径规划测试程序
 *
 * 测试内容：
 * 1. 直线移动 (lineToX, lineToY)
 * 2. 转向 (turn)
 * 3. 样条曲线 (splineTo)
 * 4. 序列动作 (SequentialAction)
 * 5. 并行动作 (ParallelAction)
 * 6. 延迟等待 (SleepAction)
 *
 * 使用说明：
 * - 在 FTC Dashboard 中可以实时查看轨迹和机器人位置
 * - 可以通过修改 TEST_CASE 来选择不同的测试场景
 * - 坐标单位：英寸 (inches)，角度单位：弧度 (radians)
 */
@Config
@Autonomous(name = "RR Path Test", group = "Test")
public class RRPathTest extends LinearOpMode {

    public TEST_CASE selectedCase = TEST_CASE.SEQUENCE_TEST;

    /**
     * 测试用例枚举
     */
    public enum TEST_CASE {
        /** 简单直线移动测试 */
        LINE_TEST,
        /** 转向测试 */
        TURN_TEST,
        /** 样条曲线测试 */
        SPLINE_TEST,
        /** 方形路径测试 */
        SQUARE_TEST,
        /** 复杂序列测试 */
        SEQUENCE_TEST,
        /** 并行动作测试 */
        PARALLEL_TEST
    }

    /**
     * 当前测试用例 - 可通过 FTC Dashboard 修改
     */


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 初始化 MecanumDrive，起始位姿为 (0, 0, 0)
        // 位姿格式: Pose2d(x, y, heading)
        // x, y 单位: 英寸, heading 单位: 弧度
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                selectedCase = TEST_CASE.LINE_TEST;
            } else if (gamepad1.b) {
                selectedCase = TEST_CASE.TURN_TEST;
            } else if (gamepad1.x) {
                selectedCase = TEST_CASE.SPLINE_TEST;
            } else if (gamepad1.y) {
                selectedCase = TEST_CASE.SQUARE_TEST;
            } else if (gamepad1.dpad_up) {
                selectedCase = TEST_CASE.SEQUENCE_TEST;
            } else if (gamepad1.dpad_down) {
                selectedCase = TEST_CASE.PARALLEL_TEST;
            }

            telemetry.addData("Status", "Select Test Case");
            telemetry.addData("Current", selectedCase.toString());
            telemetry.addData("A", "LINE_TEST");
            telemetry.addData("B", "TURN_TEST");
            telemetry.addData("X", "SPLINE_TEST");
            telemetry.addData("Y", "SQUARE_TEST");
            telemetry.addData("DPAD_UP", "SEQUENCE_TEST");
            telemetry.addData("DPAD_DOWN", "PARALLEL_TEST");
            telemetry.update();
        }



        // 根据测试用例执行不同的路径
        switch (selectedCase) {
            case LINE_TEST:
                runLineTest(drive);
                break;
            case TURN_TEST:
                runTurnTest(drive);
                break;
            case SPLINE_TEST:
                runSplineTest(drive);
                break;
            case SQUARE_TEST:
                runSquareTest(drive);
                break;
            case SEQUENCE_TEST:
                runSequenceTest(drive);
                break;
            case PARALLEL_TEST:
                runParallelTest(drive);
                break;
        }
    }

    /**
     * 测试用例 1: 直线移动
     * 机器人沿 X 轴前进 48 英寸，然后沿 Y 轴移动 24 英寸
     */
    private void runLineTest(MecanumDrive drive) {
        Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(48)           // 沿 X 轴移动到 48 英寸
                .lineToY(24)           // 沿 Y 轴移动到 24 英寸
                .lineToX(0)            // 返回 X=0
                .lineToY(0)            // 返回 Y=0
                .build();

        Actions.runBlocking(trajectory);
    }

    /**
     * 测试用例 2: 转向测试
     * 机器人原地旋转 90 度，然后 180 度，最后回到初始方向
     */
    private void runTurnTest(MecanumDrive drive) {
        Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                .turn(Math.PI / 2)     // 逆时针转 90 度
                .turn(Math.PI)         // 逆时针转 180 度
                .turn(-Math.PI / 2)    // 顺时针转 90 度
                .build();

        Actions.runBlocking(trajectory);
    }

    /**
     * 测试用例 3: 样条曲线测试
     * 机器人沿平滑曲线移动到目标位置
     *
     * 注意：RoadRunner 的 Position 路径以 Y 坐标参数化，不能使用纯水平切线方向（heading 0 或 π），
     * 因此起始段改用 lineToX 直线移动，再通过 turn 改变方向后再使用 splineTo。
     */
    private void runSplineTest(MecanumDrive drive) {
        Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                // 先直线移动到 (48, 0)，避免 splineTo 起始切线水平（heading 0）
                .lineToX(48)
                // 转向 90 度，使 heading 有 Y 分量
                .turn(Math.PI / 2)
                // 直线移动到 (48, 24)
                .lineToY(24)
                // 样条曲线移动到 (24, 48)，出口切线方向为 135 度（避免纯水平方向）
                .splineTo(new Vector2d(24, 48), 3 * Math.PI / 4)
                // 返回原点，出口切线方向为 -90 度
                .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                .build();

        Actions.runBlocking(trajectory);
    }

    /**
     * 测试用例 4: 方形路径测试
     * 机器人沿正方形路径移动，边长 36 英寸
     */
    private void runSquareTest(MecanumDrive drive) {
        double sideLength = 36;

        Action trajectory = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(sideLength)                    // 向右移动
                .lineToYConstantHeading(sideLength)     // 向上移动（保持航向）
                .lineToXConstantHeading(0)              // 向左移动（保持航向）
                .lineToYConstantHeading(0)              // 向下移动（保持航向）
                .build();

        Actions.runBlocking(trajectory);
    }

    /**
     * 测试用例 5: 复杂序列测试
     * 组合直线、转向、样条曲线和延迟
     */
    private void runSequenceTest(MecanumDrive drive) {
        Action trajectory = new SequentialAction(
                // 第一阶段：直线前进
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(36)
                        .build(),

                // 等待 0.5 秒
                new SleepAction(0.5),

                // 第二阶段：转向 90 度
                drive.actionBuilder(new Pose2d(36, 0, 0))
                        .turn(Math.PI / 2)
                        .build(),

                // 等待 0.3 秒
                new SleepAction(0.3),

                // 第三阶段：样条曲线移动
                // 注意：终点切线避免使用纯水平方向（π 或 0），改用 3π/4
                drive.actionBuilder(new Pose2d(36, 0, Math.PI / 2))
                        .splineTo(new Vector2d(48, 36), 3 * Math.PI / 4)
                        .build(),

                // 第四阶段：返回原点
                // 起始 heading 需与第三阶段终点切线一致（3π/4），避免纯水平方向
                drive.actionBuilder(new Pose2d(48, 36, 3 * Math.PI / 4))
                        .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                        .build()
        );

        Actions.runBlocking(trajectory);
    }

    /**
     * 测试用例 6: 并行动作测试
     * 演示如何组合多个动作同时执行
     * 注意：这里只是路径演示，实际应用中可以加入机械臂等动作
     *
     * 注意：RoadRunner 的 Position 路径以 Y 坐标参数化，不能使用纯水平切线方向（heading 0 或 π），
     * 因此第一段起始先直线移动再转向，避免 splineTo 起始切线水平。
     */
    private void runParallelTest(MecanumDrive drive) {
        // 构建第一段轨迹：先直线移动再转向，使 splineTo 起始切线有 Y 分量
        Action firstSegment = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(18)  // 先直线移动到 (18, 0)，避免 splineTo 起始切线水平
                .turn(Math.PI / 4)  // 转向 45 度，使 heading 有 Y 分量
                .splineTo(new Vector2d(36, 24), Math.PI / 4)
                .build();

        // 构建第二段轨迹
        Action secondSegment = drive.actionBuilder(new Pose2d(36, 24, Math.PI / 4))
                .splineTo(new Vector2d(48, 48), Math.PI / 2)
                .build();

        // 组合动作：先执行第一段，然后等待后执行第二段
        Action trajectory = new SequentialAction(
                firstSegment,
                new SleepAction(0.5),
                secondSegment,
                new SleepAction(0.5),
                // 返回原点
                drive.actionBuilder(new Pose2d(48, 48, Math.PI / 2))
                        .splineTo(new Vector2d(0, 0), -Math.PI / 2)
                        .build()
        );

        Actions.runBlocking(trajectory);
    }
}
