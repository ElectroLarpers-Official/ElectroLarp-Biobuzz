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

    // ---- Fixed-point solver parameters -------------------------------------
    // feederDelay + turretSettleTime is the horizon the launch state is projected over; flight time
    // is the lead solved inside the fixed-point loop. Kept separate: speed is not constant across both.
    public static double feederDelay = 0.1; // TODO(new robot): old-robot value
    public static double turretSettleTime = 0.03; // TODO(new robot): old-robot value
    public static double leadGain = 1; // TODO(new robot): old-robot value
    public static double solveTolerance = 0.001; // TODO(new robot): old-robot value
    public static int maxIterations = 5; // TODO(new robot): old-robot value

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

    private static final int HIST_N = 12;
    private static final double[] histT = new double[HIST_N];
    private static final double[] histV = new double[HIST_N];
    private static int histHead = 0;
    private static int histCount = 0;
    private static double filteredAlongTrackAccel = 0.0;

    // ---- Release point ---------------------------------------------------------
    /** Shooter exit above the tiles, inches. CAD: 10.4 in (steepest) to 11.6 in (flattest) release height.
     *  Only used for the flight time. */
    public static double launchHeight = 11.0;

    // ---- Tuned points (TunedShotTable) -------------------------------------------
    // Every shot comes from the tuned points: their RPM and hood are blended at the release point the robot will be
    // at (after the lead), and the flight time for the lead comes from the blended hood angle and the range.

    /** Release point -> nearest tuned point of the cell, inches (telemetry: tune more points where this is big).
     *  NaN: the cell has no tuned points. */
    public static double lastTunedNearestIn = Double.NaN;
    public static int lastTunedPointsUsed = 0;

    private static final TunedShotTable.Shot tunedShot = new TunedShotTable.Shot();
    /** This loop's shot: true when the cell has tuned points. */
    private static boolean shotOk = false;
    private static double shotFlightSec = 0.0;
    private static HiveCell lastCell = null;
    private static double lastGoalX = Double.NaN, lastGoalY = Double.NaN;

    /** Look up the tuned shot for the release point {@code range} inches from the goal, approaching along field
     *  bearing {@code bearing} (release point -> goal, radians), and its flight time. Needs goal() this loop. */
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

    /**
     * Flight time to the window centre from the hood's launch angle and the range alone: a no-drag arc through the
     * window has t = sqrt(2 (R tan(launch) - h) / g), h = window centre above the release point; drag adds a few
     * percent (TunedShotTable.FLIGHT_TIME_FACTOR). Only used for the shoot-while-moving lead. 0 (no lead) when the
     * arc can't clear the window height, which only happens closer than about 17 in (under the HIVE).
     */
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

    /** Below this projected speed (in/s) there's no lead, except while the braking model is on. */
    public static double sotmMinSpeed = 12.0; // TODO(new robot): old-robot value

    // ---- Burst comp (RevAmped's SimpleShooterMath) ------------------------------
    /** Burst comp: while feeding, re-aim the hood for the slower ball the sagging flywheel throws.
     *  The hood moves toward the launch angle that needs the least speed: steeper on flat shots, flatter on
     *  lob shots (the line between them is about 60-67 deg, depending on range). */
    public static boolean burstCompEnabled = false;
    /** true: the sag is measured, from the flywheel's real speed vs its target each loop.
     *  false: a fixed burstSpeedSag is assumed for every ball after the first. */
    public static boolean burstCompUseMeasured = true;
    /** Only with burstCompUseMeasured off: fraction of exit speed the later balls lose.
     *  A 300 RPM drop at ~3700 RPM is about 0.08. */
    public static double burstSpeedSag = 0.08;
    /** Sag smaller than this is ignored (no hood change), fraction of the target speed. */
    public static double burstCompDeadband = 0.01;
    /** Largest sag the comp will correct for, fraction (beyond this it's a stall, not a sag). */
    public static double burstCompMaxSag = 0.20;
    /** Written by Robot each loop: the flywheel's measured speed, motor ticks/s. */
    public static double flywheelTps = 0.0;
    /** Delay from feed start to applying the comp. 30 ms lands after ball 1 exits and
     *  leaves the hood servo time to move before ball 2. */
    public static double burstCompDelayMs = 30.0; // TODO(new robot): old-robot value
    /** Written by Robot each loop from its feed timer. */
    public static boolean burstCompActive = false;
    /** Hood change the burst comp applied last loop, degrees. Telemetry. */
    public static double lastBurstHoodDeltaDeg = 0.0;
    /** Measured flywheel sag last loop while feeding, fraction of target (0.08 = 8% under). Telemetry. */
    public static double lastBurstSag = 0.0;

    // ---- Ball exit speed (burst comp only) ------------------------------------------
    // 3 in flywheel, 20T GT2 on the motors -> 40T on the flywheel (motors spin 2x the flywheel).
    /** Ball exit speed / flywheel surface speed. 0.79 measured on the counter-roller shooter (2026-10-20, video +
     *  shots). Only the burst comp uses it: it turns the tuned RPM into the ball speed the sag is taken off. */
    public static double FLYWHEEL_EXIT_EFFICIENCY = 0.79;
    public static double FLYWHEEL_DIAMETER_IN = 3.0;
    public static double MOTOR_PULLEY_TEETH = 20;
    public static double FLYWHEEL_PULLEY_TEETH = 40;

    /** Motor RPM -> ball exit speed, m/s. */
    public static double ballSpeedForMotorRpm(double motorRpm) {
        double flywheelRevPerSec = motorRpm / 60.0 / (FLYWHEEL_PULLEY_TEETH / MOTOR_PULLEY_TEETH);
        return flywheelRevPerSec * Math.PI * FLYWHEEL_DIAMETER_IN * 0.0254 * FLYWHEEL_EXIT_EFFICIENCY;
    }

    /**
     * Launch angle (rad) that puts a ball of exit speed {@code v} (in/s) through the same point ({@code range} in,
     * {@code height} in above the release) as the planned shot, the one nearest {@code planned} (the planned launch
     * angle). No-drag model, so for a tuned point it is exact at the tuned speed.
     *
     * <p>Two angles reach the point when the ball is fast enough: the flat one and the lob. Taking the nearest
     * one moves the hood toward the least-speed angle: steeper for flat shots, flatter for lobs. Too slow for
     * either: the least-speed angle itself (closest the ball can get), never a wild value.
     */
    static double hoodForSpeed(double range, double height, double v, double planned) {
        double minSpeedAngle = Math.atan((height + Math.hypot(height, range)) / range);
        if (v <= 0.0 || range <= 1e-6) return minSpeedAngle;
        // range*tan(th) - g r^2 (1 + tan^2) / (2 v^2) = height  ->  a T^2 - r T + (a + h) = 0, T = tan(th)
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
    /** Flywheel motor encoder counts per motor revolution (goBILDA 5203). */
    private static final double MOTOR_TICKS_PER_REV = 28;

    // ---- Launch angle limits and hood servo -------------------------------------
    /** Launch angle range the hood can reach, degrees above horizontal (team, 2026-10-17). */
    public static double LAUNCH_MIN_DEG = 43.01765;
    public static double LAUNCH_MAX_DEG = 80.0;

    /** Hood drive: 33T gear on the servo -> 200T on the hood, goBILDA servo with 300 deg over position 0..1
     *  (Hood sets the 500-2500 us PWM range for that). Servo position 0 = the flattest launch angle,
     *  LAUNCH_MIN_DEG; position rises as the hood steepens. 80 deg launch -> about 0.747. */
    public static double HOOD_SERVO_TEETH = 33;
    public static double HOOD_GEAR_TEETH = 200;
    public static double HOOD_SERVO_RANGE_DEG = 300;
    /** Servo position at LAUNCH_MIN_DEG. Trim this if the hood doesn't sit exactly on the flat stop at 0. */
    public static double HOOD_SERVO_POS_AT_LAUNCH_MIN = 0.0;

    /** Launch angle (deg) -> hood servo position: every hood degree is 200/33 servo degrees. */
    public static double hoodServoPosition(double launchDeg) {
        double servoDeg = (launchDeg - LAUNCH_MIN_DEG) * (HOOD_GEAR_TEETH / HOOD_SERVO_TEETH);
        return HOOD_SERVO_POS_AT_LAUNCH_MIN + servoDeg / HOOD_SERVO_RANGE_DEG;
    }
    /** Inverse of hoodServoPosition: hood servo position -> launch angle, degrees. */
    public static double launchDegForHoodPosition(double position) {
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
        // Alliance picks the HIVE (red -> red HIVE, blue -> blue HIVE); field side picks the cell.
        // The Pose is cached per cell and only rebuilt when the aim point changes (e.g. edited in Panels).
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

    /** Reused result of solve: {motor ticks/s, hood servo position, turret heading deg}. Read it before the next
     *  solve call. No tuned points for the cell: ticks/s 0 and hood NaN (leave the hood where it is). */
    private static final double[] solveOut = new double[3];

    public static double[] solve(
            double robotHeading, Vector2D robotToGoalVector, Vector2D robotVel, double robotAngularVelocity) {

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

        // Where the robot will be at release: measured speed + filtered acceleration over the horizon, or the
        // Braking model while it's braking hard (revAmpedMode).
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

        // Look up the tuned shot (and its flight time) at the projected release point.
        queryTuned(projectedRange, goalTheta + projectedBearingShift);
        double tf = shotFlightSec;

        double virtualRange = projectedRange;
        double virtualX = projectedRange;
        double virtualY = 0.0;
        int iterations = 0;
        // No lead (SOTM off or too slow): the virtual release point is the projected one, which the lookup above
        // already used, so there is nothing to iterate.
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

        // The last lookup (above, or the loop's last pass) was at the final virtual release point: its RPM / hood
        // are the shot.
        double virtualBearing = goalTheta + projectedBearingShift + Math.atan2(virtualY, virtualX);
        double headingAngle = normalizeDegrees(Math.toDegrees(virtualBearing - robotHeading));

        double tps = shotOk ? Math.max(0.0, tunedShot.rpm * MOTOR_TICKS_PER_REV / 60.0) : 0.0;
        double hoodPos = shotOk ? tunedShot.hood : Double.NaN;
        requiredRPM = tps * 60.0 / MOTOR_TICKS_PER_REV;

        // Burst comp: while feeding, steer the hood so the slower (sagged) ball still goes through the point the
        // tuned shot goes through at this range.
        lastBurstHoodDeltaDeg = 0.0;
        if (burstCompEnabled && burstCompActive && shotOk && tps > 0.0) {
            double sag = burstCompUseMeasured ? 1.0 - flywheelTps / tps : burstSpeedSag;
            if (sag > burstCompDeadband) {
                sag = Math.min(sag, burstCompMaxSag);
                double th = Math.toRadians(clamp(launchDegForHoodPosition(tunedShot.hood), LAUNCH_MIN_DEG, LAUNCH_MAX_DEG));
                double v = ballSpeedForMotorRpm(tunedShot.rpm) * 39.37;   // in/s
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
