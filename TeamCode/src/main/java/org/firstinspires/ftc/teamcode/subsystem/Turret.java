package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public final class Turret {
    private final ServoImplEx turret; // null if turretServo1 isn't in the config (not plugged in yet): setPosition is a no-op
    private double LAST_POSITION = -1;
    public static double SERVO_THRESHOLD = 0.0002; // TODO(new robot): old-robot value
    // Turret pivot offset from the robot's pose point, inches (forward +, left +). From CAD on the new robot.
    public static double TURRET_FORWARD_OFFSET = 0; // aim from the robot pose itself (CAD has the axis 1.571 in behind centre; not applied)
    public static double TURRET_STRAFE_OFFSET = 0; // TODO(new robot): old-robot value
    /** Servo position with the turret straight ahead. */
    public static final double CENTER = 0.5;

    public Turret(HardwareMap hw) {
        turret = hw.tryGet(ServoImplEx.class, HardwareNames.TURRET);
        if (turret != null) {
            turret.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon MAX: full 350 deg travel over 500-2500 us
            turret.setPosition(CENTER);
        }
    }

    public static Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double turretX = robotPose.x() + TURRET_FORWARD_OFFSET * cos - TURRET_STRAFE_OFFSET * sin;
        double turretY = robotPose.y() + TURRET_FORWARD_OFFSET * sin + TURRET_STRAFE_OFFSET * cos;

        return new Pose(turretX, turretY, heading);
    }
    public static Vector2D getTurretToGoalVector(Pose turretPose, Pose goal) {
        double dx = goal.x() - turretPose.x();
        double dy = goal.y() - turretPose.y();
        return Vector2D.polar(Math.hypot(dx, dy), Math.atan2(dy, dx));
    }

    public boolean isConnected() { return turret != null; }

    public void setPosition(double pos) {
        if (turret == null) return;
        if (LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            turret.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

}