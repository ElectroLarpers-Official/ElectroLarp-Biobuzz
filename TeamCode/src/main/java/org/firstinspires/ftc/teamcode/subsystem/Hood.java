package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public final class Hood {
    public static double HOOD_DEFAULT_POS = 0.8; // TODO: confirm on the new robot
    public static double HOOD_MIN_POS = 0.05; // TODO(new robot): old-robot value
    public static double HOOD_MAX_POS = 1.0; // TODO(new robot): old-robot value
    private final Servo hood;
    private double hoodPos;

    public Hood(HardwareMap hw) {
        hood = hw.get(Servo.class, HardwareNames.HOOD);
        setPosition(HOOD_DEFAULT_POS);
    }

    public void setPosition(double position) {
        hoodPos = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        hood.setPosition(hoodPos);
    }
    public double position() { return hoodPos; }
}