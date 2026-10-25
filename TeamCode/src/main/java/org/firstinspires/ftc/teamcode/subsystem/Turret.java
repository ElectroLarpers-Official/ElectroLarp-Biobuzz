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
    /** Turret servo names this Turret looked for and didn't find in the config (telemetry). */
    private final List<String> missing = new ArrayList<>();
    private double LAST_POSITION = -1;
    public static double SERVO_THRESHOLD = 0.0002; // TODO(new robot): old-robot value
    // Turret pivot offset from the robot's pose point, inches (forward +, left +). From CAD on the new robot.
    public static double TURRET_FORWARD_OFFSET = 0; // aim from the robot pose itself (CAD has the axis 1.571 in behind centre; not applied)
    public static double TURRET_STRAFE_OFFSET = 0; // TODO(new robot): old-robot value
    /** Servo position with the turret straight ahead. */
    public static final double CENTER = 0.5;
    // turretServo1 direction, true = REVERSE. Changes in Panels apply live, on the next position write. Change it with
    // the turret at center (0.5 is the same either way): away from center, a flipped servo jumps to the mirrored
    // position and fights the others.
    public static boolean REVERSE_SERVO_1 = true;
    /** Direction each servo was last set to (null = not set yet), so a Panels change is applied once. */
    private final Boolean[] appliedReverse = new Boolean[4];

    public Turret(HardwareMap hw) {
        turret = hw.tryGet(ServoImplEx.class, HardwareNames.TURRET);
        if (turret != null) {
            turret.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon MAX: full 355 deg travel over 500-2500 us
            turret.setDirection(direction(REVERSE_SERVO_1));
            appliedReverse[0] = REVERSE_SERVO_1;
            turret.setPosition(CENTER);
        } else {
            missing.add(HardwareNames.TURRET);
        }
    }

    private static Servo.Direction direction(boolean reversed) {
        return reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD;
    }

    /** Sets a servo's direction if its REVERSE_SERVO_n flag changed since it was last set. True if it changed. */
    private boolean applyDirection(ServoImplEx servo, int i, boolean reversed) {
        if (servo == null || (appliedReverse[i] != null && appliedReverse[i] == reversed)) return false;
        servo.setDirection(direction(reversed));
        appliedReverse[i] = reversed;
        return true;
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

    /** Turret servos looked for but not in the config (telemetry). */
    public List<String> missing() { return missing; }

    public void setPosition(double pos) {
        if (turret == null) return;
        boolean changed = applyDirection(turret, 0, REVERSE_SERVO_1);
        if (changed || LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            turret.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

    // ==== TODO delete: quad turret, from here to "end quad turret" ============================================
    // turretServo2-4 on their own ports (no Y-split). Switch with Robot.TURRET_ALL_SERVOS. To go back to one servo
    // for good: delete this block, then fix every other "TODO delete" (setPositionAll -> setPosition, Turret(hw, x)
    // -> Turret(hw)).
    // turretServo2-4 directions, true = REVERSE: live like REVERSE_SERVO_1 (change them with the turret at center).
    public static boolean REVERSE_SERVO_2 = true;
    public static boolean REVERSE_SERVO_3 = false;
    public static boolean REVERSE_SERVO_4 = true;
    // Only with Turret(hw, true) and only the ones in the config (null otherwise). Written by setPositionAll, never by
    // setPosition. Separate fields, not a list, so setPositionAll writes them one after another without a loop.
    private ServoImplEx turret2;
    private ServoImplEx turret3;
    private ServoImplEx turret4;

    /** With {@code fourServos}, also gets turretServo2-4 (each one in the config, the others go in missing()) and
     *  centers them. Drive all of them with setPositionAll; setPosition still only writes turretServo1. */
    public Turret(HardwareMap hw, boolean fourServos) {
        this(hw);
        if (!fourServos) return;
        turret2 = extraServo(hw, HardwareNames.TURRET_2, REVERSE_SERVO_2);
        turret3 = extraServo(hw, HardwareNames.TURRET_3, REVERSE_SERVO_3);
        turret4 = extraServo(hw, HardwareNames.TURRET_4, REVERSE_SERVO_4);
        if (turret2 != null) appliedReverse[1] = REVERSE_SERVO_2;
        if (turret3 != null) appliedReverse[2] = REVERSE_SERVO_3;
        if (turret4 != null) appliedReverse[3] = REVERSE_SERVO_4;
    }

    /** Gets and centers one of turretServo2-4, or returns null (and records it as missing) if it isn't in the config. */
    private ServoImplEx extraServo(HardwareMap hw, String name, boolean reversed) {
        ServoImplEx servo = hw.tryGet(ServoImplEx.class, name);
        if (servo == null) {
            missing.add(name);
            return null;
        }
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Axon MAX: full 355 deg travel over 500-2500 us
        servo.setDirection(direction(reversed));
        servo.setPosition(CENTER);
        return servo;
    }

    /** Turret servos found: turretServo1 plus any of turretServo2-4 from Turret(hw, true). */
    public int servoCount() {
        return (turret != null ? 1 : 0) + (turret2 != null ? 1 : 0) + (turret3 != null ? 1 : 0) + (turret4 != null ? 1 : 0);
    }

    /** Writes turretServo1 and turretServo2-4 (from Turret(hw, true)) with the same position. With Turret(hw) or
     *  Turret(hw, false) this is the same as setPosition. */
    public void setPositionAll(double pos) {
        boolean changed = applyDirection(turret, 0, REVERSE_SERVO_1);
        changed |= applyDirection(turret2, 1, REVERSE_SERVO_2);
        changed |= applyDirection(turret3, 2, REVERSE_SERVO_3);
        changed |= applyDirection(turret4, 3, REVERSE_SERVO_4);
        if (changed || LAST_POSITION < 0 || Math.abs(pos - LAST_POSITION) > SERVO_THRESHOLD) {
            if (turret != null) turret.setPosition(pos);
            if (turret2 != null) turret2.setPosition(pos);
            if (turret3 != null) turret3.setPosition(pos);
            if (turret4 != null) turret4.setPosition(pos);
            LAST_POSITION = pos;
        }
    }

    /** Turns the PWM on turretServo2-4 off (they go limp) or back on at the last position. */
    public void setExtraServosEnabled(boolean enabled) {
        setEnabled(turret2, enabled);
        setEnabled(turret3, enabled);
        setEnabled(turret4, enabled);
    }

    private void setEnabled(ServoImplEx servo, boolean enabled) {
        if (servo == null) return;
        if (enabled) {
            servo.setPwmEnable();
            if (LAST_POSITION >= 0) servo.setPosition(LAST_POSITION);
        } else {
            servo.setPwmDisable();
        }
    }
    // ==== TODO delete: end quad turret ========================================================================

}
