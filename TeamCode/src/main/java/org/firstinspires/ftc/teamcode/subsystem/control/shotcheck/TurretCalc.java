package org.firstinspires.ftc.teamcode.subsystem.control.shotcheck;

import org.firstinspires.ftc.teamcode.util.Alliance;

// Not @Configurable on purpose: every tunable is inherited and shows once in Panels, under the pasted TurretCalc.
public class TurretCalc extends org.firstinspires.ftc.teamcode.subsystem.control.pasted.TurretCalc {

    private static boolean outOfRange = false;
    private static double aimErrorDeg = 0.0;

    public static void reset() {
        org.firstinspires.ftc.teamcode.subsystem.control.pasted.TurretCalc.reset();
        outOfRange = false;
        aimErrorDeg = 0.0;
    }

    public static double solve(double commandedAngleDeg, double robotAngularVelocityRadPerSec, Alliance alliance) {
        double servoPosition = org.firstinspires.ftc.teamcode.subsystem.control.pasted.TurretCalc.solve(
                commandedAngleDeg, robotAngularVelocityRadPerSec, alliance);

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
}
