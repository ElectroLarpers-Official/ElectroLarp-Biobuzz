package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public final class Hood {
    public static String HOOD_NAME = "hoodServo";
    public static double HOOD_DEFAULT_POS = 0.8;
    public static double HOOD_MIN_POS = 0.1;
    public static double HOOD_MAX_POS = 1.0;
    private final Servo hood;
    private double hoodPos;

    public Hood(HardwareMap hw) {
        hood = hw.get(Servo.class, HOOD_NAME);
        setPosition(HOOD_DEFAULT_POS);
    }

    /** Called every loop with the solver's hoodTime output. */
    public void setPosition(double position) {
        hoodPos = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        hood.setPosition(hoodPos);
    }
    public double position() { return hoodPos; }
}