package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.ArrayList;
import java.util.List;

@Configurable
public final class Turret {
    private final ServoImplEx turret; // null if turretServo1 isn't in the config (not plugged in yet): setPosition is a no-op
    // turretServo2-4, only with Turret(hw, true) and only the ones in the config: the 4 servos on their own ports
    // (no Y-split). Written by setPositionAll, never by setPosition.
    private final List<ServoImplEx> extraServos = new ArrayList<>();
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

    /** With {@code fourServos}, also gets turretServo2-4 (each one in the config) and centers them. Drive all of
     *  them with setPositionAll; setPosition still only writes turretServo1. */
    public Turret(HardwareMap hw, boolean fourServos) {
        this(hw);
        if (!fourServos) return;
        for (String name : new String[]{HardwareNames.TURRET_2, HardwareNames.TURRET_3, HardwareNames.TURRET_4}) {
            ServoImplEx servo = hw.tryGet(ServoImplEx.class, name);
            if (servo == null) continue;
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon MAX: full 350 deg travel over 500-2500 us
            if (name.equals(HardwareNames.TURRET_2)) servo.setDirection(Servo.Direction.REVERSE); // mounted mirrored
            servo.setPosition(CENTER);
            extraServos.add(servo);
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

    /** Turret servos found: turretServo1 plus any of turretServo2-4 from Turret(hw, true). */
    public int servoCount() { return (turret != null ? 1 : 0) + extraServos.size(); }

    public void setPosition(double pos) {
        if (turret == null) return;
        if (LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            turret.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

    /** Writes turretServo1 and turretServo2-4 (from Turret(hw, true)) with the same position. */
    public void setPositionAll(double pos) {
        if (LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            if (turret != null) turret.setPosition(pos);
            for (ServoImplEx servo : extraServos) servo.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

    /** Turns the PWM on turretServo2-4 off (they go limp) or back on at the last position. */
    public void setExtraServosEnabled(boolean enabled) {
        for (ServoImplEx servo : extraServos) {
            if (enabled) {
                servo.setPwmEnable();
                if (LAST_POSITION >= 0) servo.setPosition(LAST_POSITION);
            } else {
                servo.setPwmDisable();
            }
        }
    }

}
