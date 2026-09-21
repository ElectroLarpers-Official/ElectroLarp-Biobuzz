package org.firstinspires.ftc.teamcode.subsystem.control;

import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants.PASS_THROUGH_POINT_RADIUS;
import static org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants.SCORE_ANGLE;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Vector2D;



/**
 * Shoot-on-the-move solver. See "On-Bot Control & Dynamic Targeting"
 * (Control Lab, Module 7) for the derivation this follows.
 *
 * Step 3 - SOTM is a fixed-point problem: virtual target -> distance ->
 *          flight time -> virtual target. Iterate to tolerance, capped.
 * Step 4 - Project robot pose/velocity forward through the coast to the
 *          launch instant and solve against the PROJECTED state.
 *
 * t_d (feeder delay + turret settle) is consumed by projecting the launch
 * state forward; t_f (flight time) is the lead inside the fixed-point loop.
 * Kept separate because the robot's speed is not constant across the two
 * intervals - averaging them leads wrong.
 */
@Configurable
public class ShooterCalc {

    // ---- Fixed-point solver parameters -------------------------------------
    public static double feederDelay = 0.1;
    public static double turretSettleTime = 0.03;
    public static double leadGain = 1;
    public static double solveTolerance = 0.001;
    public static int maxIterations = 5;

    // ---- Launch-state projection -------------------------------------------
    public static final int MODE_MEASURED_ACCEL = 0;
    public static final int MODE_NONE = 2;
    public static int projectionMode = MODE_MEASURED_ACCEL;

    // ---- RevAmped braking override ------------------------------------------
    public static final int REVAMPED_OFF = 0;
    public static final int REVAMPED_AUTO = 1;
    public static final int REVAMPED_ALWAYS = 2;
    public static int revAmpedMode = REVAMPED_AUTO;
    public static double revAmpedBrakeThreshold = -60.0;
    public static double revAmpedReleaseThreshold = -10.0;
    public static boolean revAmpedBypassMinSpeed = true;
    public static boolean revAmpedActive = false;

    // ---- Accel filter --------------------------------------------------------
    public static double accelFilterAlphaDecel = 0.5;
    public static double accelFilterAlphaAccel = 0.3;
    public static double maxProjectedSpeed = 125.0;
    // Off to match the old code, whose solver call always passed a yaw rate of 0. Turn on to lead the curve.
    public static boolean compensateTurning = false;
    public static double minAngularVelocity = 0.02;
    public static double maxAngularVelocity = 6.0;
    public static double accelClamp = 400.0;
    public static double positiveAccelGain = 0.5;
    public static double slopeWindowSeconds = 0.15;
    public static double lastAlongTrackAccelRaw = 0.0;

    private static final int HIST_N = 12;
    private static final double[] histT = new double[HIST_N];
    private static final double[] histV = new double[HIST_N];
    private static int histHead = 0;
    private static int histCount = 0;
    private static double filteredAlongTrackAccel = 0.0;

    public static void resetProjectionFilter() {
        filteredAlongTrackAccel = 0.0;
        revAmpedActive = false;
        lastAlongTrackAccel = 0.0;
        lastAlongTrackAccelRaw = 0.0;
        lastProjectedSpeed = 0.0;
        histHead = 0;
        histCount = 0;
    }

    // ---- Empirical terms -------------------------------------------------
    public static double dragTimeFactor = 1;
    public static double sotmFactor = 1.0;
    public static double sotmOffset = 0.0;
    public static double verticalShift = 0.0;
    public static double sotmMinSpeed = 12.0;
    public static double zoneBlendWidth = 10.0;

    // ---- Telemetry outputs --------------------------------------------------
    public static double requiredTPS = 0.0;
    public static double requiredRPM = 0.0;
    public static int lastIterations = 0;
    public static boolean lastConverged = false;
    public static double lastFlightTime = 0.0;
    public static double lastProjectedSpeed = 0.0;
    public static double lastAlongTrackAccel = 0.0;
    public static double lastRange = 0.0;
    public static double lastProjectedRange = 0.0;
    public static double lastVirtualRange = 0.0;
    public static double lastHeadingAngle = 0.0;
    public static boolean lastSotmActive = false;

    private static final double G_IN = 32.174 * 12.0;
    // TODO: set HOOD_MIN/HOOD_MAX to the real hood's launch-angle limits.
    private static final double HOOD_MIN = Math.toRadians(46.0);
    private static final double HOOD_MAX = Math.toRadians(80);
    private static final double CLOSE_ZONE_END = 66.29;
    private static final double FAR_ZONE_START = 136.0;

    // ---- Calibration curves --------------------------------------------------
    // TODO: recalibrate targetHeight and entryAngle (and the zone boundaries above) for the BIOBUZZ target.
    public static double targetHeight(double range) {
        double poly = 0.0032 * range * range - 0.6653 * range + 66.888;
        if (range <= CLOSE_ZONE_END) return poly + 0.4;
        if (range <= FAR_ZONE_START) return poly + 0.75;
        if (range >= FAR_ZONE_START + zoneBlendWidth) return 36.0;
        double f = (range - FAR_ZONE_START) / zoneBlendWidth;
        return poly + (36.0 - poly) * f;
    }

    public static double entryAngle(double range) {
        double closeDeg = 0.6106 * range - 57.478;
        double midDeg = SCORE_ANGLE; // TODO: confirm ShooterConstants
        double farDeg = -30.0;
        double half = zoneBlendWidth / 2.0;

        double deg;
        if (range < CLOSE_ZONE_END - half) {
            deg = closeDeg;
        } else if (range < CLOSE_ZONE_END + half) {
            double f = (range - (CLOSE_ZONE_END - half)) / zoneBlendWidth;
            deg = closeDeg + (midDeg - closeDeg) * f;
        } else if (range < FAR_ZONE_START) {
            deg = midDeg;
        } else if (range < FAR_ZONE_START + zoneBlendWidth) {
            double f = (range - FAR_ZONE_START) / zoneBlendWidth;
            deg = midDeg + (farDeg - midDeg) * f;
        } else {
            deg = farDeg;
        }
        return Math.toRadians(deg);
    }

    public static double[] solveStatic(double range, double height, double entry) {
        if (range <= 1e-6) return new double[]{HOOD_MAX, 0.0};

        double hood = Math.atan(2.0 * height / range - Math.tan(entry));
        if (isNaN(hood)) hood = HOOD_MAX;
        hood = clamp(hood, HOOD_MIN, HOOD_MAX);

        double denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
        if (denom <= 0.0) {
            hood = HOOD_MAX;
            denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
            if (denom <= 0.0) return new double[]{hood, 0.0};
        }
        double speed = Math.sqrt(G_IN * range * range / denom);
        if (isNaN(speed) || Double.isInfinite(speed)) speed = 0.0;
        return new double[]{hood, speed};
    }

    public static double normalizeDegrees(double degrees) {
        if (isNaN(degrees) || Double.isInfinite(degrees)) return 0.0;
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }

    public static double flightTime(double range) {
        double[] s = solveStatic(range, targetHeight(range), entryAngle(range));
        double horizontal = s[1] * Math.cos(s[0]);
        if (horizontal < 1e-6) return Double.NaN;
        return dragTimeFactor * (range / horizontal);
    }

    /**
     * @return {requiredTPS, hoodServoPosition, turretHeadingErrorDegrees}
     */
    public static Double[] solve(
            double robotHeading, Vector2D robotToGoalVector, Vector2D robotVel,
            Vector2D robotAccel, double robotAngularVelocity) {

        double x = robotToGoalVector.magnitude() - PASS_THROUGH_POINT_RADIUS;
        if (x < 1.0) x = 1.0;

        double goalTheta = robotToGoalVector.theta();
        double currentSpeed = robotVel.magnitude();
        double velTheta = robotVel.theta();
        double horizon = feederDelay + turretSettleTime;

        double nowSec = System.nanoTime() * 1e-9;
        histT[histHead] = nowSec;
        histV[histHead] = currentSpeed;
        histHead = (histHead + 1) % HIST_N;
        if (histCount < HIST_N) histCount++;

        double aAlongV = 0.0;
        double bestDt = 0.0;
        double oldV = currentSpeed;
        for (int i = 0; i < histCount; i++) {
            int idx = (histHead - 1 - i + 2 * HIST_N) % HIST_N;
            double dt = nowSec - histT[idx];
            if (dt <= slopeWindowSeconds && dt > bestDt) {
                bestDt = dt;
                oldV = histV[idx];
            }
        }
        if (bestDt > 0.03) aAlongV = (currentSpeed - oldV) / bestDt;
        if (currentSpeed < 2.0) aAlongV = 0.0;
        if (isNaN(aAlongV) || Double.isInfinite(aAlongV)) aAlongV = 0.0;
        if (aAlongV > accelClamp) aAlongV = accelClamp;
        if (aAlongV < -accelClamp) aAlongV = -accelClamp;
        lastAlongTrackAccelRaw = aAlongV;

        if (isNaN(filteredAlongTrackAccel)) filteredAlongTrackAccel = 0.0;
        double alpha = (aAlongV < 0.0) ? accelFilterAlphaDecel : accelFilterAlphaAccel;
        filteredAlongTrackAccel += alpha * (aAlongV - filteredAlongTrackAccel);
        lastAlongTrackAccel = filteredAlongTrackAccel;

        double projectedSpeed;
        double projectedDistance;

        switch (projectionMode) {
            case MODE_MEASURED_ACCEL: {
                double a = filteredAlongTrackAccel;
                if (a > 0.0) a *= positiveAccelGain;

                if (revAmpedMode == REVAMPED_OFF || currentSpeed <= 2.0) {
                    revAmpedActive = false;
                } else if (revAmpedMode == REVAMPED_ALWAYS) {
                    revAmpedActive = true;
                } else if (revAmpedActive) {
                    revAmpedActive = a < revAmpedReleaseThreshold;
                } else {
                    revAmpedActive = a < revAmpedBrakeThreshold;
                }

                if (revAmpedActive) {
                    projectedSpeed = Braking.speedAfter(currentSpeed);
                    projectedDistance = Braking.distanceAfter(currentSpeed);
                    break;
                }

                double vEnd = currentSpeed + a * horizon;
                if (vEnd < 0.0) {
                    double tStop = (a < -1e-9) ? (currentSpeed / -a) : 0.0;
                    if (tStop < 0.0) tStop = 0.0;
                    if (tStop > horizon) tStop = horizon;
                    projectedSpeed = 0.0;
                    projectedDistance = currentSpeed * tStop + 0.5 * a * tStop * tStop;
                } else {
                    if (vEnd > maxProjectedSpeed) vEnd = maxProjectedSpeed;
                    projectedSpeed = vEnd;
                    double vAvg = 0.5 * (currentSpeed + vEnd);
                    projectedDistance = vAvg * horizon;
                }
                break;
            }
            case MODE_NONE:
            default: {
                projectedSpeed = currentSpeed;
                projectedDistance = currentSpeed * horizon;
                break;
            }
        }

        if (isNaN(projectedSpeed) || projectedSpeed < 0.0) projectedSpeed = 0.0;
        if (isNaN(projectedDistance) || projectedDistance < 0.0) projectedDistance = 0.0;
        lastProjectedSpeed = projectedSpeed;

        double omega = robotAngularVelocity;
        if (isNaN(omega) || Double.isInfinite(omega)) omega = 0.0;
        if (!compensateTurning || Math.abs(omega) < minAngularVelocity) omega = 0.0;
        if (omega > maxAngularVelocity) omega = maxAngularVelocity;
        if (omega < -maxAngularVelocity) omega = -maxAngularVelocity;

        double velThetaLaunch = velTheta + omega * horizon;
        double velThetaMid = velTheta + omega * horizon * 0.5;

        double coastAlongGoal = projectedDistance * Math.cos(velThetaMid - goalTheta);
        double coastPerpGoal = projectedDistance * Math.sin(velThetaMid - goalTheta);

        double projX = x - coastAlongGoal;
        double projY = -coastPerpGoal;
        double projectedRange = Math.hypot(projX, projY);
        if (projectedRange < 1.0) projectedRange = 1.0;
        double projectedBearingShift = Math.atan2(projY, projX);

        boolean sotmActive = projectedSpeed >= sotmMinSpeed
                || (revAmpedActive && revAmpedBypassMinSpeed);

        double coordinateTheta = velThetaLaunch - (goalTheta + projectedBearingShift);
        double parallelComponent = sotmActive ? Math.cos(coordinateTheta) * projectedSpeed : 0.0;
        double perpendicularComponent = sotmActive ? Math.sin(coordinateTheta) * projectedSpeed : 0.0;

        double tf = flightTime(projectedRange);
        if (isNaN(tf) || tf <= 0.0) tf = 0.5;

        double virtualRange = projectedRange;
        double virtualX = projectedRange;
        double virtualY = 0.0;
        int iterations = 0;
        boolean converged = false;

        for (int i = 0; i < maxIterations; i++) {
            iterations = i + 1;
            virtualX = projectedRange - leadGain * parallelComponent * tf;
            virtualY = -leadGain * perpendicularComponent * tf;
            virtualRange = Math.hypot(virtualX, virtualY);
            if (virtualRange < 1.0) virtualRange = 1.0;

            double nextTf = flightTime(virtualRange);
            if (isNaN(nextTf) || nextTf <= 0.0) break;

            if (Math.abs(nextTf - tf) < solveTolerance) {
                tf = nextTf;
                converged = true;
                break;
            }
            tf = nextTf;
        }
        lastIterations = iterations;
        lastConverged = converged;
        lastFlightTime = tf;

        double[] sol = solveStatic(virtualRange, targetHeight(virtualRange), entryAngle(virtualRange));
        double hoodAngle = sol[0];
        double launchSpeed = sol[1];
        double ballSpeedMps = launchSpeed / 39.37;

        double virtualBearing = goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));

        if (sotmActive && sotmFactor != 1.0) {
            double baseBearing = normalizeDegrees(Math.toDegrees(goalTheta - robotHeading));
            double leadDelta = normalizeDegrees(headingAngle - baseBearing);
            headingAngle = normalizeDegrees(baseBearing + sotmFactor * leadDelta);
        }

        // TODO: refit this ball-speed -> ticks/s regression on the real flywheel and ball.
        double tps = (-16.19 * ballSpeedMps * ballSpeedMps + 449.11 * ballSpeedMps - 964.9)
                + verticalShift;
        if (sotmActive) tps -= sotmOffset;
        if (isNaN(tps) || Double.isInfinite(tps) || launchSpeed <= 0.0) tps = 0.0;
        if (tps < 0.0) tps = 0.0;

        requiredTPS = tps;
        requiredRPM = tps * 60.0 / 28.0;

        double hoodDegrees = Math.toDegrees(hoodAngle);
        // TODO: refit this hood angle -> servo position mapping on the real hood.
        double hoodTime = (0.01625 * hoodDegrees) - 0.6;
        if (isNaN(hoodTime)) hoodTime = 0.0;

        lastRange = x;
        lastProjectedRange = projectedRange;
        lastVirtualRange = virtualRange;
        lastHeadingAngle = headingAngle;
        lastSotmActive = sotmActive;

        return new Double[]{tps, hoodTime, headingAngle};
    }
}