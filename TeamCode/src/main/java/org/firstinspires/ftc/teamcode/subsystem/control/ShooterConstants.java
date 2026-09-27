package org.firstinspires.ftc.teamcode.subsystem.control;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(140, 140.5); // TODO(new robot): old-robot value
    public static Pose GOAL_POS_BLUE = new Pose(3, 140.5); // TODO(new robot): old-robot value.

    // ---- BIOBUZZ HIVE aim points -------------------------------------------
    // One hardcoded (x, y) per CELL. The solver aims at whichever cell is on the
    // robot's side (see ShooterCalc.goal). Each is independent - edit live in Panels.
    //
    // Defaults come from the manual geometry: pivots at x = 59.25 (red) and
    // 84.75 (blue), y = 72, window centroid 16.49 in out from the pivot.
    // "Far" = the cell on the side away from the audience.
    public static double HIVE_RED_FAR_X = 58.4, HIVE_RED_FAR_Y = 91.4; // TODO(new robot): old-robot value
    public static double HIVE_RED_AUD_X = 58.4, HIVE_RED_AUD_Y = 51.6; // TODO(new robot): old-robot value
    public static double HIVE_BLUE_FAR_X = 84, HIVE_BLUE_FAR_Y = 91.4; // TODO(new robot): old-robot value
    public static double HIVE_BLUE_AUD_X = 84, HIVE_BLUE_AUD_Y = 51.6; // TODO(new robot): old-robot value

    /** Aim at the cell on the robot's own side of the field: robot y below the
     *  split -> audience cell, above -> far cell. Only the up-cell can be scored,
     *  and it can only be entered from its own side, so drive to the side whose
     *  cell is up. */
    public static double HIVE_SIDE_SPLIT_Y = 72.0; // TODO(new robot): old-robot value
    /** Dead band around the split, inches. Without it, a robot sitting at y = 72
     *  would flip the aim 33 in between cells every loop and slam the turret back
     *  and forth. Switches far above split+band, audience below split-band, and
     *  holds its last choice in between. */
    public static double HIVE_SIDE_HYSTERESIS = 3.0; // TODO(new robot): old-robot value

    public static double SCORE_ANGLE = -17; // TODO(new robot): old-robot value

    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon; TODO(new robot): old-robot value

    public static double RAISE_TIME = 0.15; // TODO(new robot): old-robot value

}
