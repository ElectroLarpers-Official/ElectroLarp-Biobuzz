package org.firstinspires.ftc.teamcode.subsystem.control.shotcheck;

import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants.*;
import static java.lang.Double.isNaN;

import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.subsystem.control.Braking;

// Not @Configurable on purpose: every tunable is inherited and shows once in Panels, under the pasted ShooterCalc.
public class ShooterCalc extends org.firstinspires.ftc.teamcode.subsystem.control.pasted.ShooterCalc {

    /** Inputs ShotCheck needs to rebuild the ball's flight in field coordinates. */
    public static boolean lastSolveOk = false;
    public static double lastLaunchSpeed = 0.0;          // in/s, relative to the robot
    public static double lastLaunchHoodRad = 0.0;        // launch angle the speed was solved for
    public static double lastProjectedBearing = 0.0;     // field rad, projected launch point -> goal

    /** d(ticks/s) / d(exit speed m/s) at this speed. */
    public static double tpsPerBallSpeed(double mps) {
        return useBeltFlywheelModel ? tpsForBallSpeed(1.0) : 2.0 * TPS_A * mps + TPS_B;
    }
    /** Fastest exit speed the speed model covers. The old quadratic fit turns over at 13.87 m/s; the belt model has no limit. */
    public static double maxFitBallSpeedMps() {
        return useBeltFlywheelModel ? Double.POSITIVE_INFINITY : -TPS_B / (2.0 * TPS_A);
    }
    static double gravity() { return G_IN; }

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

        // Look up the shot map at the projected release point; flightTime/targetHeight/entryAngle use it.
        queryAimTable(projectedRange, goalTheta + projectedBearingShift);
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
            queryAimTable(virtualRange, goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX));

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

        // Shot map at the final virtual release point -> score height / entry angle for the solver.
        queryAimTable(virtualRange, goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX));
        lastAimHeight = targetHeight(virtualRange);
        lastEntryDeg = Math.toDegrees(entryAngle(virtualRange));
        double[] sol = solveStatic(virtualRange, lastAimHeight, Math.toRadians(lastEntryDeg));
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
        requiredRPM = tps * 60.0 / MOTOR_TICKS_PER_REV;

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
                compHood = clamp(compHood, hoodMinRad(), hoodMaxRad());
                lastBurstHoodDeltaDeg = Math.toDegrees(compHood - hoodAngle);
                hoodAngle = compHood;
            }
        }

        double hoodDegrees = Math.toDegrees(hoodAngle);
        // Hood launch angle (deg) -> servo position (two-point calibration, see hoodServoPosition).
        double hoodTime = hoodServoPosition(hoodDegrees);

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

        lastSolveOk = launchSpeed > 0.0 && !isNaN(tf) && tf > 0.0;
        lastLaunchSpeed = launchSpeed;
        lastLaunchHoodRad = sol[0];
        lastProjectedBearing = goalTheta + projectedBearingShift;
        lastRange = x;
        lastProjectedRange = projectedRange;
        lastVirtualRange = virtualRange;
        lastHeadingAngle = headingAngle;
        lastSotmActive = sotmActive;

        return new Double[]{tps, hoodTime, headingAngle};
    }
}
