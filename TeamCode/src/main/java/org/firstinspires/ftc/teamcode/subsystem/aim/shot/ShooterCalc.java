package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants.*;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.subsystem.aim.table.HiveAimTable;
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

    private static final int HIST_N = 12;
    private static final double[] histT = new double[HIST_N];
    private static final double[] histV = new double[HIST_N];
    private static int histHead = 0;
    private static int histCount = 0;
    private static double filteredAlongTrackAccel = 0.0;

    // ---- HIVE ----------------------------------------------------------------
    // Geometry from Competition Manual V1, Figures 9-9 to 9-11: the opening is a
    // house-shaped window, 20 in wide x 14 in tall, whose outward normal points
    // 30 deg above horizontal. Without the aim table the aim point is a fixed point
    // in space, so the target height does not vary with range.

    /** Window centroid above the tiles, inches. Only used with the aim table off. */
    public static double hiveAimHeight = 65; // TODO(new robot): old-robot value
    /** Shooter exit above the tiles, inches. Target heights here are relative to the launch point.
     *  CAD: 10.4 in (steepest) to 11.6 in (flattest) release height. */
    public static double launchHeight = 11.0;
    /** Entry angle into the window, degrees. If balls rebound off the back wall and out, try -40. */
    public static double hiveEntryDeg = -50.0; // TODO(new robot): old-robot value
    /** Calibration trim on aim height, inches. Consistently short -> raise. */
    public static double hiveHeightTrim = 0.0; // TODO(new robot): old-robot value
    /** Calibration trim on entry angle, degrees (with the aim table: added to the table's entry angle).
     *  More negative = steeper, loftier shot. */
    public static double hiveEntryTrimDeg = 0.0;

    // ---- Shot-map aim table ---------------------------------------------------
    // The old solver above is kept: solveStatic(range, score height, entry angle) -> (launch angle, speed), no drag.
    // With the table on, the score height and entry angle are no longer fixed (65 in / -50 deg). At every release
    // point the shot map (drag + spin simulation of the real POLLEN, HIVE collisions, see HiveAimTable) gives the
    // launch angle and exit speed that score, and the height / entry angle below are the ones that make the
    // solver output exactly that shot from this range. Drag is folded into this "virtual" score height, so it is
    // usually above the real 58.4 in window centre and grows with range. SOTM still works: the table is read at
    // the virtual release point the lead solve ends at.

    /** Use the shot-map table for score height, entry angle and flight time. Off = old 65 in / -50 deg. */
    public static boolean USE_AIM_TABLE = true;

    private static final HiveAimTable.Shot tableShot = new HiveAimTable.Shot();
    private static boolean tableShotOk = false;
    private static HiveAimTable.Cell lastCell = null;
    private static double lastGoalX = Double.NaN, lastGoalY = Double.NaN;

    /** Telemetry: where the table was read last (virtual release point), what it said, and the solver inputs. */
    public static double lastTableX = 0.0, lastTableY = 0.0;
    public static double lastTableLaunchDeg = 0.0, lastTableSpeedMps = 0.0, lastTableWindowPct = 0.0;
    public static boolean lastTableValid = false, lastTableDependsOnOtherHive = false;
    public static double lastAimHeight = 0.0, lastEntryDeg = 0.0;

    // ---- Hand-tuned points (TunedShotTable) ----------------------------------------
    // Where a tuned point is close enough, its RPM and hood replace the shot map: they are turned into the launch
    // angle and exit speed the solver would need to output exactly that RPM and hood from this range, so SOTM,
    // the turret and ShotCheck work unchanged. Flight time is computed from the launch angle and range.

    /** Where this loop's shot came from. */
    public enum ShotSource { NONE, TUNED, MAP }
    public static ShotSource lastShotSource = ShotSource.NONE;
    /** Release point -> nearest tuned point of the cell, inches. NaN: the cell has no tuned points. */
    public static double lastTunedNearestIn = Double.NaN;
    public static int lastTunedPointsUsed = 0;
    /** No tuned point close enough and TunedShotTable.MAP_FALLBACK off: no shot here. */
    private static boolean lastNeedsTunedPoint = false;
    private static final TunedShotTable.Shot tunedShot = new TunedShotTable.Shot();

    /** True when this loop's score height / entry angle / flight time came from a tuned point or the map. */
    public static boolean aimTableInUse() { return tableShotOk; }
    /** True when this loop's shot came from tuned points. */
    public static boolean tunedShotInUse() { return aimTableInUse() && lastShotSource == ShotSource.TUNED; }
    /** True when shooting needs a tuned point here and none is close enough (MAP_FALLBACK off). */
    public static boolean needsTunedPoint() { return aimTableInUse() && lastNeedsTunedPoint; }
    public static HiveAimTable.Shot tableShot() { return tableShot; }

    /** Look up the shot for the release point {@code range} inches from the goal, approaching along field
     *  bearing {@code bearing} (release point -> goal, radians): tuned points first, then the shot map.
     *  Needs goal() to have run this loop. */
    private static void queryAimTable(double range, double bearing) {
        tableShotOk = false;
        lastShotSource = ShotSource.NONE;
        lastNeedsTunedPoint = false;
        if (lastCell == null || isNaN(lastGoalX)) return;
        double px = lastGoalX - range * Math.cos(bearing);
        double py = lastGoalY - range * Math.sin(bearing);

        boolean useTuned = TunedShotTable.USE_TUNED_POINTS;
        boolean haveTuned = useTuned && TunedShotTable.lookup(lastCell, px, py, tunedShot);
        lastTunedNearestIn = haveTuned ? tunedShot.nearestIn : Double.NaN;
        lastTunedPointsUsed = haveTuned && tunedShot.valid ? tunedShot.pointsUsed : 0;

        if (haveTuned && tunedShot.valid) {
            fillFromTuned(range, true);
        } else if (useTuned && !TunedShotTable.MAP_FALLBACK) {
            // No tuned point close enough and no map fallback: no shot. Keep the flywheel and hood near a real
            // shot (nearest tuned point, else the map) so they're ready once the robot gets to a tuned spot.
            lastNeedsTunedPoint = true;
            if (haveTuned) {
                fillFromTuned(range, false);
            } else if (USE_AIM_TABLE && HiveAimTable.lookup(lastCell, px, py, tableShot)) {
                tableShot.valid = false;
                tableShotOk = tableShot.speedMps > 0.0;
                lastShotSource = ShotSource.MAP;
            }
        } else if (USE_AIM_TABLE && HiveAimTable.lookup(lastCell, px, py, tableShot)) {
            tableShotOk = tableShot.speedMps > 0.0;
            lastShotSource = ShotSource.MAP;
        }
        if (!tableShotOk) {
            lastShotSource = ShotSource.NONE;
            return;
        }
        lastTableX = px;
        lastTableY = py;
        lastTableLaunchDeg = tableShot.launchDeg;
        lastTableSpeedMps = tableShot.speedMps;
        lastTableWindowPct = 100.0 * tableShot.halfWindowMps / Math.max(tableShot.speedMps, 1e-6);
        lastTableValid = tableShot.valid;
        lastTableDependsOnOtherHive = tableShot.dependsOnOtherHive;
    }

    /** Tuned RPM / hood -> the table shot (launch angle, exit speed, flight time) the solver reproduces them from. */
    private static void fillFromTuned(double range, boolean valid) {
        double launchDeg = clamp(launchDegForHoodPosition(tunedShot.hood), LAUNCH_MIN_DEG, LAUNCH_MAX_DEG);
        double mps = ballSpeedForMotorRpm(tunedShot.rpm);
        double th = Math.toRadians(launchDeg);
        // Flight time to the window centre from the launch angle and range alone (no exit speed needed):
        // a no-drag arc through the window has t = sqrt(2 (R tan(th) - h) / g); drag adds a few percent.
        double h = TunedShotTable.WINDOW_CENTRE_HEIGHT_IN - launchHeight;
        double rise = range * Math.tan(th) - h;
        double tf;
        if (rise > 0.0) tf = TunedShotTable.FLIGHT_TIME_FACTOR * Math.sqrt(2.0 * rise / G_IN);
        else tf = mps > 0.0 ? range / (mps * 39.37 * Math.cos(th)) : 0.0;

        tableShot.launchDeg = launchDeg;
        tableShot.speedMps = mps;
        tableShot.flightSec = tf;
        tableShot.halfWindowMps = TunedShotTable.TUNED_SPEED_WINDOW_MPS;
        tableShot.valid = valid;
        tableShot.dependsOnOtherHive = false;
        tableShot.blended = tunedShot.pointsUsed > 1;
        tableShotOk = mps > 0.0;
        lastShotSource = ShotSource.TUNED;
    }

    private static boolean hiveAimFar = true;
    private static boolean hiveSideLatched = false;

    // ---- Hood compensation (RevAmped's SimpleShooterMath) --------------------
    // Two independent hood offsets: (1) in-burst sag comp, (2) latched velocity comp.

    /** Fraction of ball exit speed the later balls in a burst lose to flywheel sag.
     *  MEASURE THIS: log flywheel TPS through a burst and map the drop at ball 2's exit
     *  back to ball speed. 0.03 is a starting estimate. */
    public static double burstSpeedSag = 0.03; // TODO(new robot): old-robot value
    public static boolean burstCompEnabled = false;
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
    public static boolean velCompEnabled = false;
    public static double velCompThreshold = 12.0;  // in/s; TODO(new robot): old-robot value
    public static double velCompToward = 0.0;      // hood signal, TUNE; TODO(new robot): old-robot value
    public static double velCompAway = 0.0;        // ADDED to Toward when retreating; TODO(new robot): old-robot value
    private static boolean velCompLatched = false;
    private static double velCompLatchedOffset = 0.0;
    /** Total offset applied last loop (velocity comp). Telemetry. */
    public static double lastHoodCompTotal = 0.0;

    public static void resetProjectionFilter() {
        velCompLatched = false;
        velCompLatchedOffset = 0.0;
        burstCompActive = false;
        hiveSideLatched = false;
        tableShotOk = false;
        lastCell = null;
        HiveAimTable.load();
        TunedShotTable.load();
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

    /** Inputs ShotCheck needs to rebuild the ball's flight in field coordinates. */
    public static boolean lastSolveOk = false;
    public static double lastLaunchSpeed = 0.0;          // in/s, relative to the robot
    public static double lastLaunchHoodRad = 0.0;        // launch angle the speed was solved for
    public static double lastProjectedBearing = 0.0;     // field rad, projected launch point -> goal

    private static final double G_IN = 32.174 * 12.0;
    static double gravity() { return G_IN; }

    // ---- Ball exit speed -> flywheel ticks/s ------------------------------------
    // 3 in flywheel, 20T GT2 on the motors -> 40T on the flywheel (motors spin 2x the flywheel),
    // goBILDA 5203 encoders (28 counts per motor rev).
    /** Ball exit speed / flywheel surface speed. 0.88 from the DECODE regression on the same hood rollers;
     *  calibrate with 240 fps video (NOTES.md, calibration step 1). */
    public static double FLYWHEEL_EXIT_EFFICIENCY = 0.88;
    public static double FLYWHEEL_DIAMETER_IN = 3.0;
    public static double MOTOR_PULLEY_TEETH = 20;
    public static double FLYWHEEL_PULLEY_TEETH = 40;
    public static double MOTOR_TICKS_PER_REV = 28;

    /** Exit speed (m/s) -> motor encoder ticks/s. */
    public static double tpsForBallSpeed(double mps) {
        double flywheelRevPerSec = mps / FLYWHEEL_EXIT_EFFICIENCY / (Math.PI * FLYWHEEL_DIAMETER_IN * 0.0254);
        return flywheelRevPerSec * (FLYWHEEL_PULLEY_TEETH / MOTOR_PULLEY_TEETH) * MOTOR_TICKS_PER_REV;
    }
    /** d(ticks/s) / d(exit speed m/s). The model is linear, so this is the same at every speed. */
    public static double tpsPerBallSpeed() {
        return tpsForBallSpeed(1.0);
    }
    /** Fastest exit speed the speed model covers. The belt model has no limit. */
    public static double maxFitBallSpeedMps() {
        return Double.POSITIVE_INFINITY;
    }

    // ---- Launch angle limits and hood servo -------------------------------------
    /** Launch angle range the hood can reach, degrees above horizontal (team, 2026-10-17). */
    public static double LAUNCH_MIN_DEG = 43.01765;
    public static double LAUNCH_MAX_DEG = 80.0;
    private static double hoodMinRad() { return Math.toRadians(LAUNCH_MIN_DEG); }
    private static double hoodMaxRad() { return Math.toRadians(LAUNCH_MAX_DEG); }

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

    /** Inverse of tpsForBallSpeed: motor RPM -> ball exit speed, m/s. Tuned points round-trip exactly whatever
     *  FLYWHEEL_EXIT_EFFICIENCY is, because the solver turns the speed back into RPM with the same model. */
    public static double ballSpeedForMotorRpm(double motorRpm) {
        double tps = motorRpm * MOTOR_TICKS_PER_REV / 60.0;
        double flywheelRevPerSec = tps / MOTOR_TICKS_PER_REV / (FLYWHEEL_PULLEY_TEETH / MOTOR_PULLEY_TEETH);
        return flywheelRevPerSec * Math.PI * FLYWHEEL_DIAMETER_IN * 0.0254 * FLYWHEEL_EXIT_EFFICIENCY;
    }

    /** Score height above the launch point, inches. Aim table: the height that makes solveStatic reproduce the
     *  map's launch angle and speed from this range (drag folded in). Otherwise the fixed window height. */
    public static double targetHeight(double range) {
        if (aimTableInUse()) {
            double th = Math.toRadians(tableShot.launchDeg);
            double v = tableShot.speedMps * 39.37;
            double c = Math.cos(th);
            return range * Math.tan(th) - G_IN * range * range / (2.0 * v * v * c * c) + hiveHeightTrim;
        }
        return hiveAimHeight - launchHeight + hiveHeightTrim;
    }

    /** Path angle at the score point, radians (negative = falling). Aim table: the slope of the map's shot at
     *  this range, so solveStatic returns the map's launch angle. Otherwise the fixed entry angle. */
    public static double entryAngle(double range) {
        if (aimTableInUse()) {
            double th = Math.toRadians(tableShot.launchDeg);
            double v = tableShot.speedMps * 39.37;
            double c = Math.cos(th);
            return Math.atan(Math.tan(th) - G_IN * range / (v * v * c * c)) + Math.toRadians(hiveEntryTrimDeg);
        }
        return Math.toRadians(hiveEntryDeg + hiveEntryTrimDeg);
    }

    public static double[] solveStatic(double range, double height, double entry) {
        if (range <= 1e-6) return new double[]{hoodMaxRad(), 0.0};

        double hood = Math.atan(2.0 * height / range - Math.tan(entry));
        if (isNaN(hood)) hood = hoodMaxRad();
        hood = clamp(hood, hoodMinRad(), hoodMaxRad());

        double denom = 2.0 * Math.pow(Math.cos(hood), 2) * (range * Math.tan(hood) - height);
        if (denom <= 0.0) {
            hood = hoodMaxRad();
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
        if (!hiveSideLatched) {
            hiveAimFar = robotY >= HIVE_SIDE_SPLIT_Y;
            hiveSideLatched = true;
        } else if (robotY > HIVE_SIDE_SPLIT_Y + HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = true;
        } else if (robotY < HIVE_SIDE_SPLIT_Y - HIVE_SIDE_HYSTERESIS) {
            hiveAimFar = false;
        }
        // Alliance picks the HIVE (red -> red HIVE, blue -> blue HIVE); field side picks the cell.
        Pose aim = ShooterConstants.hiveAimPoint(red, hiveAimFar);
        lastCell = HiveAimTable.Cell.of(red, hiveAimFar);
        lastGoalX = aim.x();
        lastGoalY = aim.y();
        return aim;
    }

    public static boolean hiveAimFar() { return hiveAimFar; }

    public static double flightTime(double range) {
        // Aim table: the simulated flight time to the window (drag makes it longer than the no-drag time below).
        if (aimTableInUse() && tableShot.flightSec > 0.0) return dragTimeFactor * tableShot.flightSec;
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