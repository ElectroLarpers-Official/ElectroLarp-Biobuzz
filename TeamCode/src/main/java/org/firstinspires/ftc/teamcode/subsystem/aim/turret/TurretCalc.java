package org.firstinspires.ftc.teamcode.subsystem.aim.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public class TurretCalc {

    /** Turret travel in degrees relative to straight ahead (soft limits, e.g. for wiring). Must stay inside what
     *  the servos can reach: +/- SERVO_RANGE_DEG/2 * 1.731 = +/- 302.9 deg with 350 deg Axon MAXes centred at 0.5.
     *  +/- 300 leaves about 3 deg of margin before the servos hit their end. Lower these if the wiring can't
     *  take that much twist. */
    public static double MIN_ANGLE = -300.0;
    public static double MAX_ANGLE = 300.0;

    public static double maxSlewNormalDegPerSec = 400; // TODO(new robot): old-robot value
    public static double maxSlewWrapDegPerSec = 540; // TODO(new robot): old-robot value

    /** Per-alliance aim trim in DEGREES, added to the commanded angle (+ = counter-clockwise). Driver 2 changes the
     *  current alliance's one live with RB / LB in TeleOp. The old robot's 15 / 10 were removed: start at 0. */
    public static double turretOffset = 0;  // RED
    public static double turretOffset2 = 0; // BLUE

    /** Servo-position trims in servo units (0..1), added to the servo signal. */
    public static double servoOffset = 0.0; // TODO: tune
    public static double wrapServoOffset = 0.0; // TODO: tune

    public static double yawFeedforwardGain = 0.145; // TODO(new robot): old-robot value

    // ---- Servo -> turret gearing (BIOBUZZ) -------------------------------------
    // 4 Axon MAXes, positional, one signal (Y-split). Each pair drives a 15T through 48T gears (48:15 speed-up); each 15T shares
    // an axle with a 33T that drives the 61T turret ring (33:61). Turret deg per servo deg = 48/15 * 33/61 = 1.731.
    public static double SERVO_GEAR_TEETH = 48, STAGE1_DRIVEN_TEETH = 15, STAGE2_DRIVE_TEETH = 33, TURRET_RING_TEETH = 61;
    /** Servo rotation over position 0..1, degrees. Axon MAX (MK2): 350 deg over 500-2500 us (Turret sets that PWM
     *  range). If the Axons were reprogrammed to a different travel, set it here. */
    public static double SERVO_RANGE_DEG = 350;
    /** Servo position with the turret straight ahead (0 deg). */
    public static double SERVO_CENTER = 0.5;
    /** Turret flips direction relative to the servo signal: set true if a positive angle turns the turret clockwise. */
    public static boolean SERVO_REVERSED = false;

    public static double turretDegPerServoDeg() {
        return (SERVO_GEAR_TEETH / STAGE1_DRIVEN_TEETH) * (STAGE2_DRIVE_TEETH / TURRET_RING_TEETH);
    }

    /** Turret angle (deg from straight ahead, CCW +) -> servo position. */
    public static double servoPositionFor(double turretDeg) {
        double servoDeg = turretDeg / turretDegPerServoDeg();
        return SERVO_CENTER + (SERVO_REVERSED ? -1 : 1) * servoDeg / SERVO_RANGE_DEG;
    }

    private static double slewedTurretAngle = Double.NaN;
    private static double lastChosenTurretAngle = Double.NaN;
    private static boolean wrapping = false;
    private static final ElapsedTime slewTimer = new ElapsedTime();

    private static boolean outOfRange = false;
    private static double aimErrorDeg = 0.0;

    public static void reset() {
        slewedTurretAngle = Double.NaN;
        lastChosenTurretAngle = Double.NaN;
        wrapping = false;
        slewTimer.reset();
        outOfRange = false;
        aimErrorDeg = 0.0;
    }

    /** Turret command -> servo position, and records outOfRange() / aimErrorDeg() for ShotCheck. */
    public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance) {
        double servoPosition = servoPosition(commandedAngleDeg, robotAngularVelocityRadPerSec, alliance);

        aimErrorDeg = Math.abs(lastChosenTurretAngle - slewedTurretAngle);

        double angularVelDeg = Math.toDegrees(robotAngularVelocityRadPerSec);
        if (Double.isNaN(angularVelDeg)) angularVelDeg = 0.0;
        double allianceOffset = (alliance == Alliance.RED) ? turretOffset : turretOffset2;
        double base = normalizeDegrees(commandedAngleDeg + allianceOffset - angularVelDeg * yawFeedforwardGain);
        outOfRange = true;
        for (int k = -1; k <= 1; k++) {
            double cand = base + 360.0 * k;
            if (cand >= MIN_ANGLE && cand <= MAX_ANGLE) outOfRange = false;
        }
        return servoPosition;
    }

    public static boolean outOfRange() { return outOfRange; }

    public static double aimErrorDeg() { return aimErrorDeg; }

    private static double servoPosition(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance) {
        double angularVelDeg = Math.toDegrees(robotAngularVelocityRadPerSec);
        if (Double.isNaN(angularVelDeg)) angularVelDeg = 0.0;
        double feedforwardOffset = angularVelDeg * yawFeedforwardGain;
        double allianceOffset = (alliance == Alliance.RED) ? turretOffset : turretOffset2;

        double rawTarget = getClosestValidTurretAngle(commandedAngleDeg + allianceOffset - feedforwardOffset);
        double targetTurretAngle = slewTurret(rawTarget);

        double servoPositionSignal = servoPositionFor(targetTurretAngle);
        servoPositionSignal = Math.max(0.0, Math.min(1.0, servoPositionSignal));

        double activeOffset = wrapping ? wrapServoOffset : servoOffset;
        return servoPositionSignal + activeOffset;
    }

    private static double normalizeDegrees(double degrees) {
        return Math.IEEEremainder(degrees, 360.0);
    }

    private static double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double base = normalizeDegrees(relativeGoalDegrees);

        double current = Double.isNaN(lastChosenTurretAngle) ? 0.0 : lastChosenTurretAngle;

        double best = Double.NaN;
        double bestCost = Double.MAX_VALUE;
        for (int k = -1; k <= 1; k++) {
            double cand = base + 360.0 * k;
            if (cand < MIN_ANGLE || cand > MAX_ANGLE) continue;
            double cost = Math.abs(cand - current);
            if (cost < bestCost) {
                bestCost = cost;
                best = cand;
            }
        }
        if (Double.isNaN(best)) {
            wrapping = false;
            lastChosenTurretAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, base));
            return lastChosenTurretAngle;
        }

        double physical = Double.isNaN(slewedTurretAngle) ? 0.0 : slewedTurretAngle;
        wrapping = Math.abs(best - physical) > 90.0;
        lastChosenTurretAngle = best;
        return best;
    }

    private static double slewTurret(double target) {
        double elapsedSec = slewTimer.seconds();
        slewTimer.reset();
        if (elapsedSec <= 0 || elapsedSec > 0.5) elapsedSec = 0.02;

        if (Double.isNaN(slewedTurretAngle)) slewedTurretAngle = target;

        double rate = wrapping ? maxSlewWrapDegPerSec : maxSlewNormalDegPerSec;
        double maxDelta = rate * elapsedSec;
        slewedTurretAngle += Range.clip(target - slewedTurretAngle, -maxDelta, maxDelta);
        return slewedTurretAngle;
    }
}