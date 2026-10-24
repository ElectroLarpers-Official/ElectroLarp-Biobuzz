package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;

@Configurable
public final class Hood {
    /** Launch angle the hood goes to at init, degrees (80 = steepest, servo ~0.747). */
    public static double START_LAUNCH_DEG = 80.0;
    /** Servo travel limits. 0.747 = 80 deg launch (33:200 gearing, 300 deg servo); a little margin above it. */
    public static double HOOD_MIN_POS = 0.0;
    public static double HOOD_MAX_POS = 0.75;
    private final Servo hood;
    private double hoodPos;

    public Hood(HardwareMap hw) {
        hood = hw.get(Servo.class, HardwareNames.HOOD);
        // goBILDA servos cover 300 deg over 500-2500 us; the SDK default (600-2400 us) would give only 270 deg.
        if (hood instanceof ServoImplEx) ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));
        hood.setDirection(Servo.Direction.REVERSE);
        setLaunchAngle(START_LAUNCH_DEG);
    }

    /** Hood to a launch angle, degrees above horizontal (43.02..80), through the 33:200 servo gearing. */
    public void setLaunchAngle(double launchDeg) {
        setPosition(ShooterCalc.hoodServoPosition(launchDeg));
    }

    public void setPosition(double position) {
        hoodPos = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        hood.setPosition(hoodPos);
    }
    public double position() { return hoodPos; }
}
