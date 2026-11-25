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
    public static double HIVE_RED_TOP_X = 59.25, HIVE_RED_TOP_Y = 86.5;
    public static double HIVE_RED_BOTTOM_X = 59.25, HIVE_RED_BOTTOM_Y = 57.5;
    public static double HIVE_BLUE_TOP_X = 84.75, HIVE_BLUE_TOP_Y = 86.5;
    public static double HIVE_BLUE_BOTTOM_X = 84.75, HIVE_BLUE_BOTTOM_Y = 57.5;

    /** Toggle: aim at the window centres from the field CAD instead of the measured points above. The tuned
     *  points were tuned with this on, so their RPM / hood assume the turret points at these. */
    public static boolean USE_MAP_AIM_POINTS = true;
    /** Turret aim points from the field CAD, Pedro inches. TOP = far cell (high y), BOTTOM = audience cell (low y).
     *  x is the window centre; y is 2 in past the window centre (88.45 / 55.55) into the cell, so the turret points
     *  at the middle of the opening from the front, side or back. Window centre height is 58.4 in. */
    public static double MAP_RED_TOP_X = 57.67, MAP_RED_TOP_Y = 88;
    public static double MAP_RED_BOTTOM_X = 57.67, MAP_RED_BOTTOM_Y = 56;
    public static double MAP_BLUE_TOP_X = 83.83, MAP_BLUE_TOP_Y = 88;
    public static double MAP_BLUE_BOTTOM_X = 83.83, MAP_BLUE_BOTTOM_Y = 56;

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
    /** Box covering both HIVEs and their frame, Pedro inches (manual 9.6). While the robot pose is inside it the
     *  solver is skipped: flywheel and hood hold the constants below and the turret keeps pointing at the cell. */
    public static boolean UNDER_HIVE_HOLD = true;
    public static double UNDER_HIVE_X_MIN = 47.27, UNDER_HIVE_X_MAX = 96.73;
    public static double UNDER_HIVE_Y_MIN = 46.7, UNDER_HIVE_Y_MAX = 97.3;
    /** Flywheel target while under the HIVE, motor RPM (a typical shot speed, so it's ready on the way out). */
    public static double UNDER_HIVE_RPM = 4200;
    /** Hood while under the HIVE, launch angle in degrees. */
    public static double UNDER_HIVE_LAUNCH_DEG = 72;

    public static boolean underHive(double x, double y) {
        return UNDER_HIVE_HOLD && x >= UNDER_HIVE_X_MIN && x <= UNDER_HIVE_X_MAX
                && y >= UNDER_HIVE_Y_MIN && y <= UNDER_HIVE_Y_MAX;
    }

    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon; TODO(new robot): old-robot value

}
