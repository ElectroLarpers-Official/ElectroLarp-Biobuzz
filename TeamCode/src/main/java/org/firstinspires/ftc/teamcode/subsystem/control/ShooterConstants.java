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
    // Measured off the Pedro 3 field (144 in, origin bottom-left, y up) in the visualizer,
    // +-2 in. TOP = higher y, BOTTOM = lower y. The field is 180-degree symmetric (not a
    // left/right mirror): RED_TOP <-> BLUE_BOTTOM and RED_BOTTOM <-> BLUE_TOP, so
    // x + x' = 144 and y + y' = 144.
    public static double HIVE_RED_TOP_X = 58.9, HIVE_RED_TOP_Y = 88.2;
    public static double HIVE_RED_BOTTOM_X = 58.9, HIVE_RED_BOTTOM_Y = 60.9;
    public static double HIVE_BLUE_TOP_X = 85.1, HIVE_BLUE_TOP_Y = 83.1;
    public static double HIVE_BLUE_BOTTOM_X = 85.1, HIVE_BLUE_BOTTOM_Y = 55.8;

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
