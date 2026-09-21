package org.firstinspires.ftc.teamcode.subsystem.control;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public class TurretCalc {

    /** Turret travel in degrees relative to straight ahead. TODO: tune to the real hardware limits. */
    public static double MIN_ANGLE = -224.75;
    public static double MAX_ANGLE = 224.75;

    public static double maxSlewNormalDegPerSec = 300;
    public static double maxSlewWrapDegPerSec = 300;

    /** Per-alliance aim trim in DEGREES, added to the commanded angle. */
    public static double turretOffset = -4;  // BLUE
    public static double turretOffset2 = 0;  // RED

    /** Servo-position trims in servo units (0..1), added to the servo signal. */
    public static double servoOffset = 0.0;
    public static double wrapServoOffset = 0.0;

    public static double yawFeedforwardGain = 0.115;

    private static double slewedTurretAngle = Double.NaN;
    private static boolean wrapping = false;
    private static final ElapsedTime slewTimer = new ElapsedTime();

    /**
     * @param commandedAngleDeg turret target relative to the robot's heading, degrees
     * @param robotAngularVelocityRadPerSec chassis yaw rate, rad/s (pass 0 if unavailable)
     * @param alliance picks which degree trim ({@link #turretOffset} or {@link #turretOffset2}) is applied
     * @return final servo position, servo offset included
     */
    public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance) {
        double angularVelDeg = Math.toDegrees(robotAngularVelocityRadPerSec);
        if (Double.isNaN(angularVelDeg)) angularVelDeg = 0.0;
        double feedforwardOffset = angularVelDeg * yawFeedforwardGain;
        double allianceOffset = (alliance == Alliance.BLUE) ? turretOffset : turretOffset2;

        double rawTarget = getClosestValidTurretAngle(commandedAngleDeg + allianceOffset - feedforwardOffset);
        double targetTurretAngle = slewTurret(rawTarget);

        double servoPositionSignal =
                0.05 + ((targetTurretAngle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)) * 0.90;
        servoPositionSignal = Math.max(0.05, Math.min(0.95, servoPositionSignal));

        double activeOffset = wrapping ? wrapServoOffset : servoOffset;
        return servoPositionSignal + activeOffset;
    }

    private static double normalizeDegrees(double degrees) {
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) return 0.0;
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }

    private static double getClosestValidTurretAngle(double relativeGoalDegrees) {
        double base = normalizeDegrees(relativeGoalDegrees);
        double current = Double.isNaN(slewedTurretAngle) ? 0.0 : slewedTurretAngle;

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
            return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, base));
        }

        wrapping = Math.abs(best - current) > 90.0;
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