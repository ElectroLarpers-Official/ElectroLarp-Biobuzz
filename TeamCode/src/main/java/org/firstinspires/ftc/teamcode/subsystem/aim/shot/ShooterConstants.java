package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class ShooterConstants {

    // ---- BIOBUZZ HIVE aim points -------------------------------------------
    // One hardcoded (x, y) per CELL. The solver aims at whichever cell is on the
    // robot's side (see ShooterCalc.goal). Each is independent - edit live in Panels.
    //
    // Measured off the Pedro 3 field (144 in, origin bottom-left, y up) in the visualizer,
    // +-2 in. TOP = higher y, BOTTOM = lower y. The field is 180-degree symmetric (not a
    // left/right mirror): RED_TOP <-> BLUE_BOTTOM and RED_BOTTOM <-> BLUE_TOP, so
    // x + x' = 144 and y + y' = 144.
    public static double HIVE_RED_TOP_X = 58.9, HIVE_RED_TOP_Y = 88.2;
    public static double HIVE_RED_BOTTOM_X = 58.9, HIVE_RED_BOTTOM_Y = 60.9;
    public static double HIVE_BLUE_TOP_X = 85.1, HIVE_BLUE_TOP_Y = 83.1;
    public static double HIVE_BLUE_BOTTOM_X = 85.1, HIVE_BLUE_BOTTOM_Y = 55.8;

    /** Toggle: aim at the window centres from the field CAD (the points the shot map was simulated against)
     *  instead of the measured points above. Keep this on while USE_AIM_TABLE is on: the table's launch angles
     *  and speeds assume the turret points at these. */
    public static boolean USE_MAP_AIM_POINTS = true;
    /** Window centres from the field CAD (shooter_analysis/NOTES.md), Pedro inches. TOP = far cell (high y),
     *  BOTTOM = audience cell (low y). Window centre height is 58.4 in. */
    public static double MAP_RED_TOP_X = 59.25, MAP_RED_TOP_Y = 88.45;
    public static double MAP_RED_BOTTOM_X = 59.25, MAP_RED_BOTTOM_Y = 55.55;
    public static double MAP_BLUE_TOP_X = 84.75, MAP_BLUE_TOP_Y = 88.45;
    public static double MAP_BLUE_BOTTOM_X = 84.75, MAP_BLUE_BOTTOM_Y = 55.55;

    /** Aim point for a cell: the CAD window centre when USE_MAP_AIM_POINTS, otherwise the measured point. */
    public static Pose hiveAimPoint(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) {
            if (red) return far ? new Pose(MAP_RED_TOP_X, MAP_RED_TOP_Y) : new Pose(MAP_RED_BOTTOM_X, MAP_RED_BOTTOM_Y);
            return far ? new Pose(MAP_BLUE_TOP_X, MAP_BLUE_TOP_Y) : new Pose(MAP_BLUE_BOTTOM_X, MAP_BLUE_BOTTOM_Y);
        }
        if (red) return far ? new Pose(HIVE_RED_TOP_X, HIVE_RED_TOP_Y) : new Pose(HIVE_RED_BOTTOM_X, HIVE_RED_BOTTOM_Y);
        return far ? new Pose(HIVE_BLUE_TOP_X, HIVE_BLUE_TOP_Y) : new Pose(HIVE_BLUE_BOTTOM_X, HIVE_BLUE_BOTTOM_Y);
    }

    /** x of hiveAimPoint(red, far), without building a Pose (ShooterCalc.goal runs every loop). */
    public static double hiveAimX(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) return red ? (far ? MAP_RED_TOP_X : MAP_RED_BOTTOM_X) : (far ? MAP_BLUE_TOP_X : MAP_BLUE_BOTTOM_X);
        return red ? (far ? HIVE_RED_TOP_X : HIVE_RED_BOTTOM_X) : (far ? HIVE_BLUE_TOP_X : HIVE_BLUE_BOTTOM_X);
    }
    /** y of hiveAimPoint(red, far). */
    public static double hiveAimY(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) return red ? (far ? MAP_RED_TOP_Y : MAP_RED_BOTTOM_Y) : (far ? MAP_BLUE_TOP_Y : MAP_BLUE_BOTTOM_Y);
        return red ? (far ? HIVE_RED_TOP_Y : HIVE_RED_BOTTOM_Y) : (far ? HIVE_BLUE_TOP_Y : HIVE_BLUE_BOTTOM_Y);
    }

    /** Aim at the alliance's own HIVE (red -> red HIVE, blue -> blue HIVE), at the cell on the robot's side
     *  of the field: robot y below the split -> audience cell, above -> far cell. Only the up-cell can be scored,
     *  and it can only be entered from its own side, so drive to the side whose
     *  cell is up. */
    public static double HIVE_SIDE_SPLIT_Y = 72.0; // TODO(new robot): old-robot value
    /** Dead band around the split, inches. Without it, a robot sitting at y = 72
     *  would flip the aim 33 in between cells every loop and slam the turret back
     *  and forth. Switches far above split+band, audience below split-band, and
     *  holds its last choice in between. */
    public static double HIVE_SIDE_HYSTERESIS = 3.0; // TODO(new robot): old-robot value

    // ---- Under the HIVE ------------------------------------------------------------
    /** Box covering both HIVEs and their frame, Pedro inches (manual 9.6 / ShotCheck obstacle values). While the
     *  robot pose is inside it the solver is skipped: flywheel and hood hold the constants below, the turret keeps
     *  pointing at the cell, and shooting is blocked. The shot map has no shots in here anyway. */
    public static boolean UNDER_HIVE_HOLD = true;
    public static double UNDER_HIVE_X_MIN = 47.27, UNDER_HIVE_X_MAX = 96.73;
    public static double UNDER_HIVE_Y_MIN = 46.7, UNDER_HIVE_Y_MAX = 97.3;
    /** Flywheel target while under the HIVE, motor RPM (a typical shot speed, so it's ready on the way out). */
    public static double UNDER_HIVE_RPM = 3300;
    /** Hood while under the HIVE, launch angle in degrees. */
    public static double UNDER_HIVE_LAUNCH_DEG = 72;

    public static boolean underHive(double x, double y) {
        return UNDER_HIVE_HOLD && x >= UNDER_HIVE_X_MIN && x <= UNDER_HIVE_X_MAX
                && y >= UNDER_HIVE_Y_MIN && y <= UNDER_HIVE_Y_MAX;
    }

    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon; TODO(new robot): old-robot value

}
