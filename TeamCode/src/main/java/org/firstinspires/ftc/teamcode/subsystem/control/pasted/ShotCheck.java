package org.firstinspires.ftc.teamcode.subsystem.control.pasted;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.Alliance;

// Not @Configurable on purpose: nothing to tune, and a second "ShotCheck" group would clash in Panels.
public final class ShotCheck {
    private ShotCheck() {}

    public static boolean check(Alliance alliance, Pose goal) { return true; }

    public static boolean allowed() { return true; }

    public static String status() { return "no shot check (pasted logic)"; }

    public static double turretToleranceDeg() { return Double.NaN; }

    public static double flywheelToleranceTps() { return Shooter.VELOCITY_TOLERANCE; }

    public static boolean ready(double flywheelVelocity, double flywheelTarget) {
        return flywheelTarget > 0 && Math.abs(flywheelVelocity - flywheelTarget) <= Shooter.VELOCITY_TOLERANCE;
    }
}
