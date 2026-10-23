package org.firstinspires.ftc.teamcode.subsystem.control;

import static org.firstinspires.ftc.teamcode.subsystem.control.HiveAimTable.Cell.RED_AUD;
import static org.firstinspires.ftc.teamcode.subsystem.control.HiveAimTable.Cell.RED_FAR;

import java.util.Arrays;
import java.util.List;

/**
 * The committed set of hand-tuned shots (see TunedShotTable). Paste the lines the ShotPointTuner prints (BACK)
 * into POINTS, commit, then clear the file on the Hub (hold BACK in the tuner).
 *
 * <p>p(cell, x, y, motor RPM, hood servo position). x, y = shooter release point (turret position), Pedro inches.
 * With TunedShotTable.ONE_CELL_FOR_ALL on (default) every point is RED_AUD (red HIVE, audience cell, nearest
 * (0, 0)) and the other three cells use them mirrored.
 */
public final class TunedShotPoints {
    private TunedShotPoints() {}

    private static final TunedShotTable.Point[] POINTS = {
            // Tuned 2026-10-22 (ShotPointTuner, 13.5 -> 11.65 V). Three points from that session are commented out at
            // the end of this list (with the reason); re-tune those spots before using them.
            p(RED_AUD, 51.7, 37.6, 3476, 0.572),
            p(RED_AUD, 81.6, 23.7, 3682, 0.500),
            p(RED_AUD, 78.7, 9.9, 3726, 0.330),
            p(RED_AUD, 54.8, 17.1, 3719, 0.435),
            p(RED_AUD, 30.2, 21.7, 3888, 0.335),
            p(RED_AUD, 48.5, 43.1, 3751, 0.622),
            p(RED_AUD, 66.3, 36.9, 3609, 0.624),
            p(RED_AUD, 83.6, 20.2, 3656, 0.401),
            p(RED_AUD, 97.7, 8.2, 4041, 0.391),   // check pose: y close to the wall
            p(RED_AUD, 118.9, 16.8, 3746, 0.303),
            p(RED_AUD, 133.7, 13.9, 3946, 0.303),

            // NOT USED - accidental save: hood 0.5855 is exactly the shot map's 72 deg prediction (saved in follow
            // mode before adjusting). Only 3.5 in from (81.6, 23.7) but ~600 RPM lower, and needs a flywheel
            // efficiency of 0.88 to score where every other point needs ~0.77. Would pull blends there low.
            p(RED_AUD, 78.3, 24.9, 3082, 0.586),

            // NOT USED - pose was off: a release point can't be below y = 0 (about y = 9 with the robot against the
            // wall), so the pose had drifted 10-17 in when these were saved. The RPM / hood may be right, but they're
            // stored at the wrong spot. Check the pose against the wall, then re-tune that corner.
            // p(RED_AUD, 114.6, -0.9, 4242, 0.132),
            // p(RED_AUD, 127.3, -8.2, 4342, 0.127),
    };

    static List<TunedShotTable.Point> points() { return Arrays.asList(POINTS); }

    private static TunedShotTable.Point p(HiveAimTable.Cell cell, double x, double y, double rpm, double hood) {
        return new TunedShotTable.Point(cell, x, y, rpm, hood, false);
    }
}
