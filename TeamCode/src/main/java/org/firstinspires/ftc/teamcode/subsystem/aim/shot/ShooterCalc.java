package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants.*;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.subsystem.aim.table.HiveCell;
import org.firstinspires.ftc.teamcode.subsystem.aim.table.TunedShotTable;
import org.firstinspires.ftc.teamcode.util.Alliance;



@Configurable
public class ShooterCalc {


    public static double feederDelay = 0.1;
    public static double turretSettleTime = 0.03;
    public static double leadGain = 1;
    public static double solveTolerance = 0.001;
    public static int maxIterations = 5;

    // ---- RevAmped braking override ------------------------------------------
    public static final int REVAMPED_OFF = 0;
    public static final int REVAMPED_AUTO = 1;
    public static final int REVAMPED_ALWAYS = 2;
    public static int revAmpedMode = REVAMPED_AUTO;
    public static double revAmpedBrakeThreshold = -100.0;
    public static double revAmpedReleaseThreshold = -75.0;
    public static boolean revAmpedBypassMinSpeed = true;
    public static boolean revAmpedActive = false;

    // ---- Accel filter --------------------------------------------------------
    public static double accelFilterAlphaDecel = 0.5;
    public static double accelFilterAlphaAccel = 0.3;
    public static double maxProjectedSpeed = 125.0;
    public static boolean compensateTurning = true;
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

    // ---- Release point ---------------------------------------------------------
    public static double launchHeight = 11.0;

    // ---- Tuned points (TunedShotTable) -------------------------------------------
    public static double lastTunedNearestIn = Double.NaN;
    public static int lastTunedPointsUsed = 0;

    private static final TunedShotTable.Shot tunedShot = new TunedShotTable.Shot();
    private static boolean shotOk = false;
    private static double shotFlightSec = 0.0;
    private static HiveCell lastCell = null;
    private static double lastGoalX = Double.NaN, lastGoalY = Double.NaN;
    private static void queryTuned(double range, double bearing) {
        shotOk = false;
        if (lastCell == null || isNaN(lastGoalX)) return;
        double px = lastGoalX - range * Math.cos(bearing);
        double py = lastGoalY - range * Math.sin(bearing);

        shotOk = TunedShotTable.lookup(lastCell, px, py, tunedShot);
        lastTunedNearestIn = shotOk ? tunedShot.nearestIn : Double.NaN;
        lastTunedPointsUsed = shotOk ? tunedShot.pointsUsed : 0;
        shotFlightSec = shotOk ? flightTime(range, tunedShot.hood) : 0.0;
    }

    public static double flightTime(double range, double hoodPosition) {
        double th = Math.toRadians(clamp(launchDegForHoodPosition(hoodPosition), LAUNCH_MIN_DEG, LAUNCH_MAX_DEG));
        double rise = range * Math.tan(th) - (TunedShotTable.WINDOW_CENTRE_HEIGHT_IN - launchHeight);
        return rise > 0.0 ? TunedShotTable.FLIGHT_TIME_FACTOR * Math.sqrt(2.0 * rise / G_IN) : 0.0;
    }

    private static boolean hiveAimFar = true;
    private static boolean hiveSideLatched = false;

    public static void resetProjectionFilter() {
        burstCompActive = false;
        hiveSideLatched = false;
        shotOk = false;
        lastCell = null;
        TunedShotTable.load();
        filteredAlongTrackAccel = 0.0;
        revAmpedActive = false;
        lastAlongTrackAccel = 0.0;
        lastAlongTrackAccelRaw = 0.0;
        lastProjectedSpeed = 0.0;
        histHead = 0;
        histCount = 0;
    }


    public static double sotmMinSpeed = 12.0;

    // ---- Burst comp (RevAmped's SimpleShooterMath) ------------------------------
    public static boolean burstCompEnabled = false;
    public static boolean burstCompUseMeasured = true;
    public static double burstSpeedSag = 0.08;
    public static double burstCompDeadband = 0.01;
    public static double burstCompMaxSag = 0.20;
    public static double flywheelTps = 0.0;
    public static double burstCompDelayMs = 30.0;
    public static boolean burstCompActive = false;
    public static double lastBurstHoodDeltaDeg = 0.0;
    public static double lastBurstSag = 0.0;

    // ---- Ball exit speed (burst comp only) ------------------------------------------
    public static double FLYWHEEL_EXIT_EFFICIENCY = 0.79;
    public static double FLYWHEEL_DIAMETER_IN = 3.0;
    public static double MOTOR_PULLEY_TEETH = 20;
    public static double FLYWHEEL_PULLEY_TEETH = 40;

    public static double ballSpeedForMotorRpm(double motorRpm) {
        double flywheelRevPerSec = motorRpm / 60.0 / (FLYWHEEL_PULLEY_TEETH / MOTOR_PULLEY_TEETH);
        return flywheelRevPerSec * Math.PI * FLYWHEEL_DIAMETER_IN * 0.0254 * FLYWHEEL_EXIT_EFFICIENCY;
    }
    static double hoodForSpeed(double range, double height, double v, double planned) {
        double minSpeedAngle = Math.atan((height + Math.hypot(height, range)) / range);
        if (v <= 0.0 || range <= 1e-6) return minSpeedAngle;
        double a = G_IN * range * range / (2.0 * v * v);
        double disc = range * range - 4.0 * a * (a + height);
        if (disc < 0.0) return minSpeedAngle;
        double sq = Math.sqrt(disc);
        double flat = Math.atan((range - sq) / (2.0 * a));
        double lob = Math.atan((range + sq) / (2.0 * a));
        return (planned <= minSpeedAngle) ? flat : lob;
    }

    // ---- Telemetry outputs --------------------------------------------------
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
    private static final double MOTOR_TICKS_PER_REV = 28;

    // ---- Launch angle limits and hood servo -------------------------------------
    public static double LAUNCH_MIN_DEG = 43.01765;
    public static double LAUNCH_MAX_DEG = 80.0;
    public static boolean FIXED_LAUNCH = true;
    public static double FIXED_LAUNCH_DEG = 67;

    public static double HOOD_SERVO_TEETH = 33;
    public static double HOOD_GEAR_TEETH = 200;
    public static double HOOD_SERVO_RANGE_DEG = 300;
    public static double HOOD_SERVO_POS_AT_LAUNCH_MIN = 0.0;

    public static double hoodServoPosition(double launchDeg) {
        double servoDeg = (launchDeg - LAUNCH_MIN_DEG) * (HOOD_GEAR_TEETH / HOOD_SERVO_TEETH);
        return HOOD_SERVO_POS_AT_LAUNCH_MIN + servoDeg / HOOD_SERVO_RANGE_DEG;
    }
    public static double launchDegForHoodPosition(double position) {
        if (FIXED_LAUNCH) return FIXED_LAUNCH_DEG;
        double servoDeg = (position - HOOD_SERVO_POS_AT_LAUNCH_MIN) * HOOD_SERVO_RANGE_DEG;
        return LAUNCH_MIN_DEG + servoDeg * (HOOD_SERVO_TEETH / HOOD_GEAR_TEETH);
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
        if (!hiveSideLatched) {
            hiveAimFar = robotY >= HIVE_SIDE_SPLIT_Y;
            hiveSideLatched = true;
        } else if (robotY > HIVE_SIDE_SPLIT_Y + HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = true;
        } else if (robotY < HIVE_SIDE_SPLIT_Y - HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = false;
        }
        int i = (red ? 0 : 2) + (hiveAimFar ? 1 : 0);
        double ax = ShooterConstants.hiveAimX(red, hiveAimFar), ay = ShooterConstants.hiveAimY(red, hiveAimFar);
        Pose aim = goalCache[i];
        if (aim == null || aim.x() != ax || aim.y() != ay) {
            aim = new Pose(ax, ay);
            goalCache[i] = aim;
        }
        lastCell = HiveCell.of(red, hiveAimFar);
        lastGoalX = ax;
        lastGoalY = ay;
        return aim;
    }
    private static final Pose[] goalCache = new Pose[4];

    public static boolean hiveAimFar() { return hiveAimFar; }

    private static final double[] solveOut = new double[3];

    // ---- Meta Infinity's shoot-while-moving ---------------------------------------------------------------------

    public static boolean USE_META_SOTM = true;

    private static double[] solveMeta(double robotHeading, double x, double goalTheta, Vector2D robotVel) {
        queryTuned(x, goalTheta);
        double tf = shotFlightSec;

        double speed = robotVel.magnitude();
        double along = speed * Math.cos(robotVel.theta() - goalTheta);
        double across = speed * Math.sin(robotVel.theta() - goalTheta);

        double virtualX = x - along * tf;
        double virtualY = -across * tf;
        double virtualRange = Math.max(1.0, Math.hypot(virtualX, virtualY));
        double virtualBearing = goalTheta + Math.atan2(virtualY, virtualX);

        queryTuned(virtualRange, virtualBearing);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));
        double tps = shotOk ? Math.max(0.0, tunedShot.rpm * MOTOR_TICKS_PER_REV / 60.0) : 0.0;
        double hoodPos = shotOk ? tunedShot.hood : Double.NaN;

        requiredRPM = tps * 60.0 / MOTOR_TICKS_PER_REV;
        lastBurstHoodDeltaDeg = 0.0;
        lastBurstSag = 0.0;
        revAmpedActive = false;
        lastIterations = 1;
        lastConverged = true;
        lastFlightTime = tf;
        lastProjectedSpeed = speed;
        lastAlongTrackAccel = 0.0;
        lastRange = x;
        lastProjectedRange = x;
        lastVirtualRange = virtualRange;
        lastHeadingAngle = headingAngle;
        lastSotmActive = true;

        solveOut[0] = tps;
        solveOut[1] = hoodPos;
        solveOut[2] = headingAngle;
        return solveOut;
    }

    // ---- Our shoot-while-moving (USE_META_SOTM = false) ---------------------------------------------------------

    public static double[] solve(
            double robotHeading, Vector2D robotToGoalVector, Vector2D robotVel, double robotAngularVelocity) {

        double x = robotToGoalVector.magnitude() - PASS_THROUGH_POINT_RADIUS;
        if (x < 1.0) x = 1.0;

        double goalTheta = robotToGoalVector.theta();
        if (USE_META_SOTM) return solveMeta(robotHeading, x, goalTheta, robotVel);
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
        } else {
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

        queryTuned(projectedRange, goalTheta + projectedBearingShift);
        double tf = shotFlightSec;

        double virtualRange = projectedRange;
        double virtualX = projectedRange;
        double virtualY = 0.0;
        int iterations = 0;
        boolean converged = !sotmActive;

        for (int i = 0; i < maxIterations && sotmActive; i++) {
            iterations = i + 1;
            virtualX = projectedRange - leadGain * parallelComponent * tf;
            virtualY = -leadGain * perpendicularComponent * tf;
            virtualRange = Math.hypot(virtualX, virtualY);
            if (virtualRange < 1.0) virtualRange = 1.0;
            queryTuned(virtualRange, goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX));

            double nextTf = shotFlightSec;
            if (nextTf <= 0.0) break;

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

        double virtualBearing = goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));

        double tps = shotOk ? Math.max(0.0, tunedShot.rpm * MOTOR_TICKS_PER_REV / 60.0) : 0.0;
        double hoodPos = shotOk ? tunedShot.hood : Double.NaN;
        requiredRPM = tps * 60.0 / MOTOR_TICKS_PER_REV;

        lastBurstHoodDeltaDeg = 0.0;
        if (burstCompEnabled && !FIXED_LAUNCH && burstCompActive && shotOk && tps > 0.0) {
            double sag = burstCompUseMeasured ? 1.0 - flywheelTps / tps : burstSpeedSag;
            if (sag > burstCompDeadband) {
                sag = Math.min(sag, burstCompMaxSag);
                double th = Math.toRadians(clamp(launchDegForHoodPosition(tunedShot.hood), LAUNCH_MIN_DEG, LAUNCH_MAX_DEG));
                double v = ballSpeedForMotorRpm(tunedShot.rpm) * 39.37;
                double c = Math.cos(th);
                double height = virtualRange * Math.tan(th) - G_IN * virtualRange * virtualRange / (2.0 * v * v * c * c);
                double comp = clamp(hoodForSpeed(virtualRange, height, v * (1.0 - sag), th),
                        Math.toRadians(LAUNCH_MIN_DEG), Math.toRadians(LAUNCH_MAX_DEG));
                lastBurstHoodDeltaDeg = Math.toDegrees(comp - th);
                hoodPos = hoodServoPosition(Math.toDegrees(comp));
            }
        }
        lastBurstSag = burstCompActive && tps > 0.0 ? 1.0 - flywheelTps / tps : 0.0;

        lastRange = x;
        lastProjectedRange = projectedRange;
        lastVirtualRange = virtualRange;
        lastHeadingAngle = headingAngle;
        lastSotmActive = sotmActive;

        solveOut[0] = tps;
        solveOut[1] = hoodPos;
        solveOut[2] = headingAngle;
        return solveOut;
    }
}
