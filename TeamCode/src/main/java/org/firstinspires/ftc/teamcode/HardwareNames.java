package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class HardwareNames {
    private HardwareNames() {}

    // Drivetrain (Expansion Hub)
    public static String FRONT_LEFT = "frontLeft";
    public static String FRONT_RIGHT = "frontRight";
    public static String BACK_LEFT = "backLeft";
    public static String BACK_RIGHT = "backRight";

    // Localization
    public static String PINPOINT = "pinpoint";
    public static String IMU = "imu";

    // Motors (Control Hub)
    public static String INTAKE = "intake";
    /** Transfer motor, belted to the intake; driven by Intake together with it (optional in the config). */
    public static String TRANSFER = "transfer";
    public static String SHOOTER = "shooter";    // flywheel motor with the encoder
    public static String SHOOTER_2 = "shooter2";

    // Servos (Control Hub)
    public static String HOOD = "hoodServo";
    public static String LATCH = "stopperServo";

    // 4 servos on one signal wire; Y-split into one device: turretServo1 on Control Hub servo port 0
    public static String TURRET = "turretServo1";
    // Only when the 4 turret servos are on their own ports (no Y-split): read by Turret(hw, true) and written by
    // Turret.setPositionAll (test/TurretTest). Robot uses Turret(hw) + setPosition, i.e. turretServo1 only.
    public static String TURRET_2 = "turretServo2";
    public static String TURRET_3 = "turretServo3";
    public static String TURRET_4 = "turretServo4";

    // TODO: add the Limelight and Arducam names here (and in the CLAUDE.md table) when vision is added back.
}
