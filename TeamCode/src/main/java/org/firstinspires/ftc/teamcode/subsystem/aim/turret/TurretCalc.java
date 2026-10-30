package org.firstinspires.ftc.teamcode.subsystem.aim.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public class TurretCalc {

    /** Turret travel in degrees relative to straight ahead (soft limits, e.g. for wiring). Must stay inside what
     *  the servos can reach: +/- SERVO_RANGE_DEG/2 * 1.731 = +/- 307.3 deg with 355 deg Axon MAXes centred at 0.5.
     *  +/- 300 leaves about 7 deg of margin before the servos hit their end. Lower these if the wiring can't
     *  take that much twist. */
    public static double MIN_ANGLE = -300.0;
    public static double MAX_ANGLE = 300.0;

    /** Turret speed limit for the command, deg/s: normally / on a big swing (more than 90 deg to go). */
    public static double maxSlewNormalDegPerSec = 400; // TODO(new robot): old-robot value
    public static double maxSlewWrapDegPerSec = 540; // TODO(new robot): old-robot value
    /** Turret acceleration limit for the command, deg/s^2. The command speeds up at this rate and slows down early
     *  enough to stop ON the target instead of arriving at full speed and stopping dead, which is what makes the
     *  turret overshoot and wobble after a snap. 0 = no limit (old behaviour: full speed from the first loop).
     *  5000 (simulated through the real solver): braking-SOTM aim jumps of 25-55 deg are within 2 deg 120-180 ms
     *  after the brake starts (no limit: 90-135 ms), well before the robot stops; a 1500 deg/s^2 robot spin is
     *  followed within 0.4 deg. Lower it if the turret still wobbles after snaps, raise it if braking shots are late. */
    public static double maxAccelDegPerSec2 = 5000;
    /** Smoothing of the measured target speed (0..1, higher = less smoothing). The command follows a smoothly
     *  moving target (the robot turning, driving past the HIVE) at the target's own speed, so the acceleration
     *  limit doesn't make it lag. */
    public static double targetRateFilter = 0.5;

    /** Per-alliance aim trim in DEGREES, added to the commanded angle (+ = counter-clockwise). Driver 2 changes the
     *  current alliance's one live with RB / LB in TeleOp. The old robot's 15 / 10 were removed: start at 0. */
    public static double turretOffset = 0;  // RED
    public static double turretOffset2 = 0; // BLUE

    /** Servo-position trims in servo units (0..1), added to the servo signal. */
    public static double servoOffset = 0.0; // TODO: tune
    public static double wrapServoOffset = 0.0; // TODO: tune

    public static double yawFeedforwardGain = 0.145; // TODO(new robot): old-robot value

    // ---- Servo -> turret gearing (BIOBUZZ) -------------------------------------
    // 4 Axon MAXes, positional, all at the same position (four ports; turretServo2 reversed, see Turret). Each pair drives a 15T through 48T gears (48:15 speed-up); each 15T shares
    // an axle with a 33T that drives the 61T turret ring (33:61). Turret deg per servo deg = 48/15 * 33/61 = 1.731.
    public static double SERVO_GEAR_TEETH = 48, STAGE1_DRIVEN_TEETH = 15, STAGE2_DRIVE_TEETH = 33, TURRET_RING_TEETH = 61;
    /** Servo rotation over position 0..1, degrees: the travel the Axon MAXes are programmed to (355, team
     *  2026-10-23), over 500-2500 us (Turret sets that PWM range). */
    public static double SERVO_RANGE_DEG = 355;
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
    /** Command speed, deg/s (acceleration-limited). */
    private static double slewVelocity = 0.0;
    /** Measured target speed, deg/s, and the target last loop. */
    private static double targetRate = 0.0;
    private static double lastSlewTarget = Double.NaN;
    private static double lastChosenTurretAngle = Double.NaN;
    private static boolean wrapping = false;
    private static final ElapsedTime slewTimer = new ElapsedTime();

    private static boolean outOfRange = false;
    private static double aimErrorDeg = 0.0;

    public static void reset() {
        slewedTurretAngle = Double.NaN;
        slewVelocity = 0.0;
        targetRate = 0.0;
        lastSlewTarget = Double.NaN;
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

    /**
     * Moves the command toward {@code target} within the speed and acceleration limits.
     *
     * <p>The command's wanted speed is the target's own speed plus a catch-up speed of sqrt(2 * accel * gap), the
     * fastest it can go and still stop on the target. Its speed changes by at most accel * dt per loop. So:
     * a jump (the braking model engaging, a new cell) is a smooth speed-up, cruise and slow-down onto the new aim;
     * a smoothly moving target is followed at its own speed with no lag. A target that moves faster than the
     * turret could follow in one loop counts as a jump: its speed isn't fed forward, or the command would overshoot.
     */
    private static double slewTurret(double target) {
        double elapsedSec = slewTimer.seconds();
        slewTimer.reset();
        if (elapsedSec <= 0 || elapsedSec > 0.5) elapsedSec = 0.02;

        if (Double.isNaN(slewedTurretAngle)) {
            slewedTurretAngle = target;
            slewVelocity = 0.0;
            targetRate = 0.0;
            lastSlewTarget = target;
            return slewedTurretAngle;
        }

        double maxSpeed = wrapping ? maxSlewWrapDegPerSec : maxSlewNormalDegPerSec;

        // Target speed: only smooth motion counts; a jump bigger than the turret could make in one loop doesn't.
        double moved = Double.isNaN(lastSlewTarget) ? 0.0 : target - lastSlewTarget;
        lastSlewTarget = target;
        if (Math.abs(moved) > maxSpeed * elapsedSec) targetRate = 0.0;
        else targetRate += targetRateFilter * (moved / elapsedSec - targetRate);

        if (maxAccelDegPerSec2 <= 0) {   // speed limit only (old behaviour)
            double maxDelta = maxSpeed * elapsedSec;
            double step = Range.clip(target - slewedTurretAngle, -maxDelta, maxDelta);
            slewedTurretAngle += step;
            slewVelocity = step / elapsedSec;
            return slewedTurretAngle;
        }

        double accel = maxAccelDegPerSec2;
        double gap = target - slewedTurretAngle;
        double wanted = targetRate + Math.signum(gap) * Math.sqrt(2.0 * accel * Math.abs(gap));
        wanted = Range.clip(wanted, -maxSpeed, maxSpeed);
        slewVelocity += Range.clip(wanted - slewVelocity, -accel * elapsedSec, accel * elapsedSec);

        double step = slewVelocity * elapsedSec;
        if (Math.signum(step) == Math.signum(gap) && Math.abs(step) >= Math.abs(gap)) {
            // Reaches the target this loop: land on it and carry on at the target's speed.
            slewedTurretAngle = target;
            slewVelocity = Range.clip(targetRate, -maxSpeed, maxSpeed);
        } else {
            slewedTurretAngle += step;
        }
        return slewedTurretAngle;
    }
}