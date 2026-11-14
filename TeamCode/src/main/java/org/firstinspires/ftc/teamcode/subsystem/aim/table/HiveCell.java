package org.firstinspires.ftc.teamcode.subsystem.aim.table;

/** Which HIVE cell a shot goes into: alliance HIVE (red / blue) x side (audience / far). */
public enum HiveCell {
    RED_AUD, RED_FAR, BLUE_AUD, BLUE_FAR;

    public static HiveCell of(boolean red, boolean far) {
        if (red) return far ? RED_FAR : RED_AUD;
        return far ? BLUE_FAR : BLUE_AUD;
    }
}
