package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.Collections;
import java.util.Map;

public final class NoDrivetrain implements Drivetrain {
    @Override public void drive(DrivePowers powers, boolean useBrake) {}
    @Override public double maxScaling(DrivePowers a, DrivePowers b) { return 1.0; }
    @Override public void stop() {}
    @Override public void stop(boolean useBrake) {}
    @Override public Map<String, Object> debug() { return Collections.<String, Object>singletonMap("drivetrain", "not connected"); }
    @Override public double interpolateVelocity(double a, double b, double c) { return 0.0; }
}
