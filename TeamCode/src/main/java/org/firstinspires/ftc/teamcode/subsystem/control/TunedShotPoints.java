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
            // p(RED_AUD, 30.0, 20.0, 3450, 0.512),
    };

    static List<TunedShotTable.Point> points() { return Arrays.asList(POINTS); }

    private static TunedShotTable.Point p(HiveAimTable.Cell cell, double x, double y, double rpm, double hood) {
        return new TunedShotTable.Point(cell, x, y, rpm, hood, false);
    }
}
