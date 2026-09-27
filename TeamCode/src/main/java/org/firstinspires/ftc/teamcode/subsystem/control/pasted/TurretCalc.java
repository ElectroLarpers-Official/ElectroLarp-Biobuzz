package org.firstinspires.ftc.teamcode.subsystem.control.pasted;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public class TurretCalc {

    /** Turret travel in degrees relative to straight ahead. */
    public static double MIN_ANGLE = -224.75; // TODO(new robot): old-robot value
    public static double MAX_ANGLE = 224.75; // TODO(new robot): old-robot value

    public static double maxSlewNormalDegPerSec = 400; // TODO(new robot): old-robot value
    public static double maxSlewWrapDegPerSec = 540; // TODO(new robot): old-robot value

    /** Per-alliance aim trim in DEGREES, added to the commanded angle. */
    public static double turretOffset = 15;  // RED; TODO(new robot): old-robot value
    public static double turretOffset2 = 10; // BLUE; TODO(new robot): old-robot value

    /** Servo-position trims in servo units (0..1), added to the servo signal. */
    public static double servoOffset = 0.0; // TODO: tune
    public static double wrapServoOffset = 0.0; // TODO: tune

    public static double yawFeedforwardGain = 0.145; // TODO(new robot): old-robot value

    protected static double slewedTurretAngle = Double.NaN;
    protected static double lastChosenTurretAngle = Double.NaN;
    protected static boolean wrapping = false;
    private static final ElapsedTime slewTimer = new ElapsedTime();

    public static void reset() {
        slewedTurretAngle = Double.NaN;
        lastChosenTurretAngle = Double.NaN;
        wrapping = false;
        slewTimer.reset();
    }

    public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance) {
        double angularVelDeg = Math.toDegrees(robotAngularVelocityRadPerSec);
        if (Double.isNaN(angularVelDeg)) angularVelDeg = 0.0;
        double feedforwardOffset = angularVelDeg * yawFeedforwardGain;
        double allianceOffset = (alliance == Alliance.RED) ? turretOffset : turretOffset2;

        double rawTarget = getClosestValidTurretAngle(commandedAngleDeg + allianceOffset - feedforwardOffset);
        double targetTurretAngle = slewTurret(rawTarget);

        double servoPositionSignal =
                0.05 + ((targetTurretAngle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)) * 0.90; // TODO(new robot): old-robot value
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal)); // TODO(new robot): old-robot value

        double activeOffset = wrapping ? wrapServoOffset : servoOffset;
        return servoPositionSignal + activeOffset;
    }

    protected static double normalizeDegrees(double degrees) {
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