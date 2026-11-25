package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class ShooterConstants {

    // ---- BIOBUZZ HIVE aim points -------------------------------------------
    public static double HIVE_RED_TOP_X = 59.25, HIVE_RED_TOP_Y = 86.5;
    public static double HIVE_RED_BOTTOM_X = 59.25, HIVE_RED_BOTTOM_Y = 57.5;
    public static double HIVE_BLUE_TOP_X = 84.75, HIVE_BLUE_TOP_Y = 86.5;
    public static double HIVE_BLUE_BOTTOM_X = 84.75, HIVE_BLUE_BOTTOM_Y = 57.5;

    public static boolean USE_MAP_AIM_POINTS = true;
    public static double MAP_RED_TOP_X = 57.67, MAP_RED_TOP_Y = 88;
    public static double MAP_RED_BOTTOM_X = 57.67, MAP_RED_BOTTOM_Y = 56;
    public static double MAP_BLUE_TOP_X = 83.83, MAP_BLUE_TOP_Y = 88;
    public static double MAP_BLUE_BOTTOM_X = 83.83, MAP_BLUE_BOTTOM_Y = 56;

    public static Pose hiveAimPoint(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) {
            if (red) return far ? new Pose(MAP_RED_TOP_X, MAP_RED_TOP_Y) : new Pose(MAP_RED_BOTTOM_X, MAP_RED_BOTTOM_Y);
            return far ? new Pose(MAP_BLUE_TOP_X, MAP_BLUE_TOP_Y) : new Pose(MAP_BLUE_BOTTOM_X, MAP_BLUE_BOTTOM_Y);
        }
        if (red) return far ? new Pose(HIVE_RED_TOP_X, HIVE_RED_TOP_Y) : new Pose(HIVE_RED_BOTTOM_X, HIVE_RED_BOTTOM_Y);
        return far ? new Pose(HIVE_BLUE_TOP_X, HIVE_BLUE_TOP_Y) : new Pose(HIVE_BLUE_BOTTOM_X, HIVE_BLUE_BOTTOM_Y);
    }

    public static double hiveAimX(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) return red ? (far ? MAP_RED_TOP_X : MAP_RED_BOTTOM_X) : (far ? MAP_BLUE_TOP_X : MAP_BLUE_BOTTOM_X);
        return red ? (far ? HIVE_RED_TOP_X : HIVE_RED_BOTTOM_X) : (far ? HIVE_BLUE_TOP_X : HIVE_BLUE_BOTTOM_X);
    }
    public static double hiveAimY(boolean red, boolean far) {
        if (USE_MAP_AIM_POINTS) return red ? (far ? MAP_RED_TOP_Y : MAP_RED_BOTTOM_Y) : (far ? MAP_BLUE_TOP_Y : MAP_BLUE_BOTTOM_Y);
        return red ? (far ? HIVE_RED_TOP_Y : HIVE_RED_BOTTOM_Y) : (far ? HIVE_BLUE_TOP_Y : HIVE_BLUE_BOTTOM_Y);
    }

    public static double HIVE_SIDE_SPLIT_Y = 72.0;
    public static double HIVE_SIDE_HYSTERESIS = 3.0;

    // ---- Under the HIVE ------------------------------------------------------------
    public static boolean UNDER_HIVE_HOLD = true;
    public static double UNDER_HIVE_X_MIN = 47.27, UNDER_HIVE_X_MAX = 96.73;
    public static double UNDER_HIVE_Y_MIN = 46.7, UNDER_HIVE_Y_MAX = 97.3;
    public static double UNDER_HIVE_RPM = 4200;
    public static double UNDER_HIVE_LAUNCH_DEG = 72;

    public static boolean underHive(double x, double y) {
        return UNDER_HIVE_HOLD && x >= UNDER_HIVE_X_MIN && x <= UNDER_HIVE_X_MAX
                && y >= UNDER_HIVE_Y_MIN && y <= UNDER_HIVE_Y_MAX;
    }

    public static double PASS_THROUGH_POINT_RADIUS = 0;

}
