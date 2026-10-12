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
    public static String INTAKE = "intakeMotor";
    public static String TRANSFER_1 = "transferMotor";
    public static String TRANSFER_2 = "transferMotor2";
    public static String SHOOTER = "launchingmotor";

    // Servos (Control Hub)
    public static String HOOD = "hoodServo";
    public static String LATCH = "stopperServo";

    // 4 servos on one signal wire; Y-split into one device: turretServo1 on Control Hub servo port 0
    public static String TURRET = "turretServo1";

    // TODO: add the Limelight and Arducam names here (and in the CLAUDE.md table) when vision is added back.
}
