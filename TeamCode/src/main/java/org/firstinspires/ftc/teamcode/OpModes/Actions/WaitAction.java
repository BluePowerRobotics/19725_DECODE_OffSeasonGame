package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/**
 * 等待Action：暂停指定时间后继续
 */
public class WaitAction implements Action {
    private final long durationMs;
    private long startTimeMs = -1;

    public WaitAction(long durationMs) {
        this.durationMs = durationMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (startTimeMs < 0) {
            startTimeMs = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - startTimeMs;
        packet.put("WaitAction", "Waiting... (" + (durationMs - elapsed) + "ms remaining)");
        return elapsed < durationMs;
    }
}