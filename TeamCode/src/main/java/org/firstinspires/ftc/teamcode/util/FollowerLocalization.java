package org.firstinspires.ftc.teamcode.util;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.autotune.AutoTune;

/**
 * {@code PulseFollower.update()} without the driving, for TeleOp after {@code Robot.startTeleOpDrive()}
 * (the RevAmped arcade drive owns the drive motors, so the follower must not write them).
 *
 * <p>Calling {@code follower.localizer.update()} alone would leave Pulse's extras stale. This runs the same
 * per-loop work {@code PulseFollower.update()} does, in the same order, except the drive step:
 * <ol>
 *   <li>{@code localizer.update()}: pose and velocity (what {@code Follower.update()} does before driving)</li>
 *   <li>{@code acceleration().update(velocity())}: so {@code follower.acceleration()} /
 *       {@code getAcceleration()} stay live</li>
 *   <li>{@code AutoTune.samplePose(pose())} when no AutoTune recording is running (Pulse's visualizer feed;
 *       the recording branch only runs while following an autotuned path, which can't happen here)</li>
 *   <li>{@code loop().record(start)} and {@code AutoTune.loopRate(...)}: so {@code follower.loop()} stays live</li>
 * </ol>
 * Call it once per loop, after the bulk caches are cleared, in place of {@code follower.update()}.
 */
public final class FollowerLocalization {
    private FollowerLocalization() {}

    public static void update(PulseFollower follower) {
        long start = System.nanoTime();
        follower.localizer.update();
        follower.acceleration().update(follower.velocity());
        if (AutoTune.active() == null) AutoTune.samplePose(follower.pose());
        follower.loop().record(start);
        AutoTune.loopRate(follower.loop().hz(), follower.loop().worstHz());
    }
}
