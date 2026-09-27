package org.firstinspires.ftc.teamcode.subsystem.control.pasted;

import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants.*;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.subsystem.control.Braking;
import org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;



@Configurable
public class ShooterCalc {

    // ---- Fixed-point solver parameters -------------------------------------
    // feederDelay + turretSettleTime is the horizon the launch state is projected over; flight time
    // is the lead solved inside the fixed-point loop. Kept separate: speed is not constant across both.
    public static double feederDelay = 0.1; // TODO(new robot): old-robot value
    public static double turretSettleTime = 0.03; // TODO(new robot): old-robot value
    public static double leadGain = 1; // TODO(new robot): old-robot value
    public static double solveTolerance = 0.001; // TODO(new robot): old-robot value
    public static int maxIterations = 5; // TODO(new robot): old-robot value

    // ---- Launch-state projection -------------------------------------------
    public static final int MODE_MEASURED_ACCEL = 0;
    public static final int MODE_NONE = 2;
    public static int projectionMode = MODE_MEASURED_ACCEL;

    // ---- RevAmped braking override ------------------------------------------
    public static final int REVAMPED_OFF = 0;
    public static final int REVAMPED_AUTO = 1;
    public static final int REVAMPED_ALWAYS = 2;
    public static int revAmpedMode = REVAMPED_AUTO;
    /** Measured decel (in/s^2) that ENGAGES the braking model. Clear of the ~-52 float coast. */
    public static double revAmpedBrakeThreshold = -100.0; // TODO(new robot): old-robot value
    /** Measured decel (in/s^2) at which it RELEASES. Latched far from the engage threshold so
     *  one stop is treated as one event instead of stepping the aim point between models. */
    public static double revAmpedReleaseThreshold = -75.0; // TODO(new robot): old-robot value
    public static boolean revAmpedBypassMinSpeed = true;
    public static boolean revAmpedActive = false;

    // ---- Accel filter --------------------------------------------------------
    public static double accelFilterAlphaDecel = 0.5; // TODO(new robot): old-robot value
    public static double accelFilterAlphaAccel = 0.3; // TODO(new robot): old-robot value
    public static double maxProjectedSpeed = 125.0; // TODO(new robot): old-robot value
    /** Rotate the velocity direction forward through the horizon so the lead points where the robot
     *  will be heading at launch. Without it, at 30 deg/s the hit rate on a 7in target fell to 50%;
     *  with it, 99%. Robot passes the follower's real yaw rate (the old code passed 0, so this never ran there). */
    public static boolean compensateTurning = true;
    public static double minAngularVelocity = 0.02; // TODO(new robot): old-robot value
    public static double maxAngularVelocity = 6.0; // TODO(new robot): old-robot value
    public static double accelClamp = 400.0; // TODO(new robot): old-robot value
    public static double positiveAccelGain = 0.5; // TODO(new robot): old-robot value
    public static double slopeWindowSeconds = 0.15; // TODO(new robot): old-robot value
    public static double lastAlongTrackAccelRaw = 0.0;

    protected static final int HIST_N = 12;
    protected static final double[] histT = new double[HIST_N];
    protected static final double[] histV = new double[HIST_N];
    protected static int histHead = 0;
    protected static int histCount = 0;
    protected static double filteredAlongTrackAccel = 0.0;

    // ---- BIOBUZZ HIVE mode ------------------------------------------------
    // Geometry from Competition Manual V1, Figures 9-9 to 9-11: the opening is a
    // house-shaped window, 20 in wide x 14 in tall, whose outward normal points
    // 30 deg above horizontal. The aim point is a fixed point in space, so unlike
    // the DECODE regression the target height does not vary with range.

    /** Toggle: aim at the BIOBUZZ HIVE instead of the DECODE goal. */
    public static boolean hiveMode = true;
    public static void toggleHiveMode() { hiveMode = !hiveMode; }

    /** Window centroid above the tiles, inches. */
    public static double hiveAimHeight = 65; // TODO(new robot): old-robot value
    /** Shooter exit above the tiles, inches. Target heights here are relative to the launch point. */
    public static double launchHeight = 14.0; // TODO(new robot): old-robot value
    /** Entry angle into the window, degrees. If balls rebound off the back wall and out, try -40. */
    public static double hiveEntryDeg = -50.0; // TODO(new robot): old-robot value
    /** Calibration trim on aim height, inches. Consistently short -> raise. */
    public static double hiveHeightTrim = 0.0; // TODO(new robot): old-robot value

    private static boolean hiveAimFar = true;
    private static boolean hiveSideLatched = false;

    // ---- Hood compensation (RevAmped's SimpleShooterMath) --------------------
    // Two independent hood offsets: (1) in-burst sag comp, (2) latched velocity comp.

    /** Fraction of ball exit speed the later balls in a burst lose to flywheel sag.
     *  MEASURE THIS: log flywheel TPS through a burst and map the drop at ball 2's exit
     *  back to ball speed. 0.03 is a starting estimate. */
    public static double burstSpeedSag = 0.03; // TODO(new robot): old-robot value
    public static boolean burstCompEnabled = true;
    /** Delay from feed start to applying the comp. 30 ms lands after ball 1 exits and
     *  leaves the hood servo time to move before ball 2. */
    public static double burstCompDelayMs = 30.0; // TODO(new robot): old-robot value
    /** Written by Robot each loop from its feed timer. */
    public static boolean burstCompActive = false;
    /** Hood change the burst comp applied last loop, degrees. Telemetry. */
    public static double lastBurstHoodDeltaDeg = 0.0;

    /** Velocity comp, LATCHED: direction is decided when speed first crosses the threshold
     *  and held until it drops back under. Offsets stay 0 until moving shots show a
     *  consistent direction-dependent miss that stationary shots do not. */
    public static boolean velCompEnabled = true;
    public static double velCompThreshold = 12.0;  // in/s; TODO(new robot): old-robot value
    public static double velCompToward = 0.0;      // hood signal, TUNE; TODO(new robot): old-robot value
    public static double velCompAway = 0.0;        // ADDED to Toward when retreating; TODO(new robot): old-robot value
    protected static boolean velCompLatched = false;
    protected static double velCompLatchedOffset = 0.0;
    /** Total offset applied last loop (velocity comp). Telemetry. */
    public static double lastHoodCompTotal = 0.0;

    public static void resetProjectionFilter() {
        velCompLatched = false;
        velCompLatchedOffset = 0.0;
        burstCompActive = false;
        hiveSideLatched = false;
        filteredAlongTrackAccel = 0.0;
        revAmpedActive = false;
        lastAlongTrackAccel = 0.0;
        lastAlongTrackAccelRaw = 0.0;
        lastProjectedSpeed = 0.0;
        histHead = 0;
        histCount = 0;
    }

    // ---- Empirical terms -------------------------------------------------
    public static double dragTimeFactor = 1; // TODO(new robot): old-robot value
    public static double sotmFactor = 1.0; // TODO(new robot): old-robot value
    public static double sotmOffset = 0.0; // TODO(new robot): old-robot value
    public static double verticalShift = 0.0; // TODO(new robot): old-robot value
    /** verticalShift nudge per press. Reserved for driver 2 controls, which are not added yet. */
    public static double verticalShiftStep = 50.0; // TODO(new robot): old-robot value
    public static double sotmMinSpeed = 12.0; // TODO(new robot): old-robot value
    public static double zoneBlendWidth = 2; // TODO(new robot): old-robot value

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

    protected static final double G_IN = 32.174 * 12.0;

    // Ball speed (m/s) -> flywheel ticks/s: tps = A*v^2 + B*v + C, fit on the old robot.
    protected static final double TPS_A = -16.19; // TODO(new robot): old-robot value
    protected static final double TPS_B = 449.11; // TODO(new robot): old-robot value
    protected static final double TPS_C = -964.9; // TODO(new robot): old-robot value

    public static double tpsForBallSpeed(double mps) { return TPS_A * mps * mps + TPS_B * mps + TPS_C; }
    protected static final double HOOD_MIN = Math.toRadians(46.0); // TODO(new robot): old-robot value
    protected static final double HOOD_MAX = Math.toRadians(80); // TODO(new robot): old-robot value
    private static final double CLOSE_ZONE_END = 66.29; // TODO(new robot): old-robot value
    private static final double FAR_ZONE_START = 123.0; // TODO(new robot): old-robot value

    public static double targetHeight(double range) {
        if (hiveMode) return hiveAimHeight - launchHeight + hiveHeightTrim;
        double poly = 0.0032 * range * range - 0.6653 * range + 66.888; // TODO(new robot): old-robot value
        if (range <= CLOSE_ZONE_END) return poly + 0.3;
        if (range <= FAR_ZONE_START) return poly;
        if (range >= FAR_ZONE_START + zoneBlendWidth) return 34.8; // TODO(new robot): old-robot value
        double f = (range - FAR_ZONE_START) / zoneBlendWidth;
        return (poly + (36.0 - poly) * f) - 1.8; // TODO(new robot): old-robot value
    }

    public static double entryAngle(double range) {
        if (hiveMode) return Math.toRadians(hiveEntryDeg);
        double closeDeg = 0.6106 * range - 57.478; // TODO(new robot): old-robot value
        double midDeg = SCORE_ANGLE;
        double farDeg = -30.0; // TODO(new robot): old-robot value
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

    public static Pose goal(Alliance alliance, double robotY) {
        boolean red = alliance == Alliance.RED;
        if (!hiveMode) {
            hiveSideLatched = false;
            return red ? GOAL_POS_RED : GOAL_POS_BLUE;
        }
        if (!hiveSideLatched) {
            hiveAimFar = robotY >= HIVE_SIDE_SPLIT_Y;
            hiveSideLatched = true;
        } else if (robotY > HIVE_SIDE_SPLIT_Y + HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = true;
        } else if (robotY < HIVE_SIDE_SPLIT_Y - HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = false;
        }
        if (red) {
            return hiveAimFar ? new Pose(HIVE_RED_FAR_X, HIVE_RED_FAR_Y) : new Pose(HIVE_RED_AUD_X, HIVE_RED_AUD_Y);
        }
        return hiveAimFar ? new Pose(HIVE_BLUE_FAR_X, HIVE_BLUE_FAR_Y) : new Pose(HIVE_BLUE_AUD_X, HIVE_BLUE_AUD_Y);
    }

    public static boolean hiveAimFar() { return hiveAimFar; }

    public static double flightTime(double range) {
        double[] s = solveStatic(range, targetHeight(range), entryAngle(range));
        double horizontal = s[1] * Math.cos(s[0]);
        if (horizontal < 1e-6) return Double.NaN;
        return dragTimeFactor * (range / horizontal);
    }

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

        double tps = tpsForBallSpeed(ballSpeedMps) + verticalShift;
        if (sotmActive) tps -= sotmOffset;
        if (isNaN(tps) || Double.isInfinite(tps) || launchSpeed <= 0.0) tps = 0.0;
        if (tps < 0.0) tps = 0.0;

        requiredTPS = tps;
        requiredRPM = tps * 60.0 / 28.0; // TODO(new robot): old-robot value

        lastBurstHoodDeltaDeg = 0.0;
        if (burstCompEnabled && burstCompActive && burstSpeedSag > 0.0 && launchSpeed > 0.0) {
            double r = virtualRange;
            double h = targetHeight(virtualRange);
            double v = launchSpeed * (1.0 - burstSpeedSag);
            double a = G_IN * r * r / (2.0 * v * v);
            double disc = r * r - 4.0 * a * (a + h);
            double compHood = Double.NaN;
            if (disc >= 0.0) {
                double tLow = Math.atan((r - Math.sqrt(disc)) / (2.0 * a));
                double tHigh = Math.atan((r + Math.sqrt(disc)) / (2.0 * a));
                compHood = (Math.abs(tLow - hoodAngle) <= Math.abs(tHigh - hoodAngle)) ? tLow : tHigh;
            } else if (v * v - 2.0 * G_IN * h > 0.0) {
                compHood = Math.atan(v / Math.sqrt(v * v - 2.0 * G_IN * h));
            }
            if (!isNaN(compHood)) {
                compHood = clamp(compHood, HOOD_MIN, HOOD_MAX);
                lastBurstHoodDeltaDeg = Math.toDegrees(compHood - hoodAngle);
                hoodAngle = compHood;
            }
        }

        double hoodDegrees = Math.toDegrees(hoodAngle);
        // Hood launch angle (deg) -> servo position, fit on the old robot.
        double hoodTime = (0.01625 * hoodDegrees) - 0.6; // TODO(new robot): old-robot value

        if (velCompEnabled) {
            if (!velCompLatched && currentSpeed > velCompThreshold) {
                boolean toward = Math.cos(velTheta - goalTheta) > 0.0;
                velCompLatchedOffset = toward ? velCompToward : velCompToward + velCompAway;
                velCompLatched = true;
            } else if (velCompLatched && currentSpeed <= velCompThreshold) {
                velCompLatched = false;
                velCompLatchedOffset = 0.0;
            }
        } else {
            velCompLatched = false;
            velCompLatchedOffset = 0.0;
        }
        lastHoodCompTotal = velCompLatchedOffset;
        hoodTime += lastHoodCompTotal;
        if (isNaN(hoodTime)) hoodTime = 0.0;

        lastRange = x;
        lastProjectedRange = projectedRange;
        lastVirtualRange = virtualRange;
        lastHeadingAngle = headingAngle;
        lastSotmActive = sotmActive;

        return new Double[]{tps, hoodTime, headingAngle};
    }
}