package org.firstinspires.ftc.teamcode.subsystem.control.shotcheck;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystem.control.HiveAimTable;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public final class ShotCheck {

    public enum Result {
        OK("ok"),
        NO_SOLUTION("no trajectory reaches the aim point"),
        PAST_FLYWHEEL_FIT("too far: past the flywheel speed fit"),
        FLYWHEEL_LIMIT("too far: flywheel can't spin that fast"),
        TURRET_LIMIT("turret can't rotate that far"),
        TOO_CLOSE("too close: ball still rising at the opening"),
        BEHIND_WINDOW("behind the cell opening"),
        TOO_EDGE_ON("too far to the side of the cell opening"),
        WINDOW_TOO_NARROW("opening looks narrower than the ball from here"),
        BLOCKED_OTHER_HIVE("other alliance's HIVE is in the way"),
        BLOCKED_FRAME("HIVE frame is in the way"),
        NOT_ON_MAP("shot map: no scoring shot from here"),
        UNDER_HIVE("under the HIVE: holding fixed RPM / hood, no shot"),
        TOO_TIGHT("shot map: shot too tight (speed window not over the minimum)"),
        NO_TUNED_POINT("no tuned shot point close enough (MAP_FALLBACK is off)");

        public final String text;
        Result(String text) { this.text = text; }
    }

    // ---- Ball and shooter ----------------------------------------------------
    /** Ball diameter, inches. DECODE ball on the old robot. New robot shoots POLLEN: 2.8. */
    public static double BALL_DIAMETER = 2.8; // POLLEN, 71.1 mm
    /** Highest flywheel speed the motors can hold, motor ticks/s: 5800 rpm (free speed tested at 12 V) x 28 / 60.
     *  TODO(new robot): measure (full power, read telemetry). */
    public static double MAX_FLYWHEEL_TPS = 2700;
    /** Aim table: share of the map's exit-speed window the flywheel may use, leaving the rest for hood error. */
    public static double TABLE_SPEED_WINDOW_USE = 0.8;
    /** Aim table: a shot is only allowed if its exit-speed window (+/- m/s around the planned speed) is MORE than
     *  this. The map's windows come in 0.025 m/s steps, so 0.075 refuses spots with +/-0.075 or less (about 22% of
     *  scoring spots); the typical window is +/- 0.22 m/s (about 3.8%). 0 = no minimum. */
    public static double MIN_SPEED_WINDOW_MPS = 0.075;
    /** Aim table: let the robot shoot from spots that score only with the other HIVE tilted one particular way. */
    public static boolean ALLOW_OTHER_HIVE_DEPENDENT = true;
    /** Clearance kept on EACH side of the ball inside the opening, inches, for everything
     *  the tolerances don't model (hood servo error, ball variation, pose error). */
    public static double APERTURE_MARGIN = 0.5; // TODO: tune

    // ---- CELL opening (manual 9.6.2 + the solver's HIVE model) ------------------
    public static double WINDOW_WIDTH = 20.0;   // across the field (x)
    public static double WINDOW_HEIGHT = 14.0;  // along the tilted opening
    /** The opening's outward normal points this far above horizontal, degrees. */
    public static double WINDOW_NORMAL_ELEVATION_DEG = 30.0;
    /** Most edge-on the ball may arrive, degrees from the opening's normal. The flat-opening
     *  model gets optimistic near 90, where the cell's side walls start to matter. */
    public static double MAX_INCIDENCE_DEG = 70.0;

    // ---- Obstacles (Pedro inches) --------------------------------------------
    // TODO: field geometry below is from Competition Manual V1 9.6, Team Update 01 and the Pedro visualizer
    // image (1080 px = 144 in, audience at y = 0, red on the low-x side); image values are approximate, check them on the CAD.
    /** HIVE outlines from the visualizer image: cells + pivot assembly, both ends. */
    public static double RED_HIVE_X_MIN = 48.0, RED_HIVE_X_MAX = 69.3;
    public static double BLUE_HIVE_X_MIN = 75.3, BLUE_HIVE_X_MAX = 96.0;
    public static double HIVE_Y_MIN = 46.7, HIVE_Y_MAX = 97.3;
    /** "Bottom of HIVE above TILES", Team Update 01. */
    public static double HIVE_BOTTOM = 30.6;
    /** Up-cell apex (manual figure 9-10, as used by the solver's HIVE model). */
    public static double HIVE_TOP = 65.6;
    /** Frame: two triangular end plates at x = 72 +/- 24.73 (49.46 wide), base 38.95 deep,
     *  apex at the pivot axis, 43.95 in up (manual 9.6.1). */
    public static double FRAME_X_MIN = 47.27, FRAME_X_MAX = 96.73;
    public static double FRAME_CENTER_Y = 72.0;
    public static double FRAME_HALF_DEPTH = 19.475;
    public static double FRAME_APEX_HEIGHT = 43.95;

    /** Samples along the flight for the obstacle check. */
    public static int FLIGHT_SAMPLES = 40;

    // ---- DECODE mode (no window model) ----------------------------------------
    public static double DECODE_TURRET_TOLERANCE_DEG = 2.0; // TODO: placeholder, no old-robot value exists
    public static double DECODE_FLYWHEEL_TOLERANCE_TPS = 100; // TODO: placeholder, no old-robot value exists

    // ---- Outputs -----------------------------------------------------------
    public static Result lastResult = Result.NO_SOLUTION;
    /** Turret aim error the opening leaves room for, degrees. */
    public static double lastTurretToleranceDeg = 0.0;
    /** Flywheel speed error the opening leaves room for, ticks/s. */
    public static double lastFlywheelToleranceTps = 0.0;
    /** Angle between the ball's path and the opening's normal, degrees. Telemetry. */
    public static double lastIncidenceDeg = 0.0;

    private ShotCheck() {}

    public static boolean allowed() { return lastResult == Result.OK; }

    public static String status() { return lastResult.text; }

    public static double turretToleranceDeg() { return lastTurretToleranceDeg; }
    public static double flywheelToleranceTps() { return lastFlywheelToleranceTps; }

    public static boolean ready(double flywheelVelocity, double flywheelTarget) {
        return allowed()
                && TurretCalc.aimErrorDeg() <= lastTurretToleranceDeg
                && Math.abs(flywheelVelocity - flywheelTarget) <= lastFlywheelToleranceTps;
    }

    /** Under the HIVE: solver skipped, shooting blocked. */
    public static void blockUnderHive() {
        lastTurretToleranceDeg = 0.0;
        lastFlywheelToleranceTps = 0.0;
        lastIncidenceDeg = 0.0;
        lastResult = Result.UNDER_HIVE;
    }

    public static boolean check(Alliance alliance, Pose goal) {
        lastTurretToleranceDeg = 0.0;
        lastFlywheelToleranceTps = 0.0;
        lastIncidenceDeg = 0.0;
        lastResult = evaluate(alliance, goal, !TurretCalc.outOfRange());
        return allowed();
    }

    private static Result evaluate(Alliance alliance, Pose goal, boolean turretInRange) {
        if (ShooterCalc.aimTableInUse()) return evaluateTable(turretInRange);
        double v = ShooterCalc.lastLaunchSpeed;
        double hood = ShooterCalc.lastLaunchHoodRad;
        double tf = ShooterCalc.lastFlightTime;
        double range = ShooterCalc.lastProjectedRange;
        double virtualRange = ShooterCalc.lastVirtualRange;

        if (!ShooterCalc.lastSolveOk || v <= 0 || tf <= 0) return Result.NO_SOLUTION;
        double vMps = v / 39.37;
        if (vMps > ShooterCalc.maxFitBallSpeedMps()) return Result.PAST_FLYWHEEL_FIT;
        if (ShooterCalc.requiredTPS > MAX_FLYWHEEL_TPS) return Result.FLYWHEEL_LIMIT;

        if (!turretInRange) return Result.TURRET_LIMIT;

        double bearing = ShooterCalc.lastProjectedBearing;
        double cb = Math.cos(bearing), sb = Math.sin(bearing);
        double launchX = goal.x() - range * cb;
        double launchY = goal.y() - range * sb;
        double z0 = ShooterCalc.launchHeight;
        double vz0 = v * Math.sin(hood);
        double vh = range / tf;
        double vzEnd = vz0 - ShooterCalc.gravity() * tf;
        double speedEnd = Math.hypot(vh, vzEnd);
        double dx = vh * cb / speedEnd, dy = vh * sb / speedEnd, dz = vzEnd / speedEnd;
        double descent = Math.atan2(-vzEnd, vh);

        double lateralRoom, verticalRoom;
        if (ShooterCalc.hiveMode) {
            if (vzEnd >= 0) return Result.TOO_CLOSE;

            double side = ShooterCalc.hiveAimFar() ? 1.0 : -1.0;
            double elev = Math.toRadians(WINDOW_NORMAL_ELEVATION_DEG);
            double ny = side * Math.cos(elev), nz = Math.sin(elev);
            double wy = -side * Math.sin(elev), wz = Math.cos(elev);

            double cosIncidence = -(dy * ny + dz * nz);
            lastIncidenceDeg = Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, cosIncidence))));
            if (cosIncidence <= 0 || dy * side >= 0) return Result.BEHIND_WINDOW;
            if (lastIncidenceDeg > MAX_INCIDENCE_DEG) return Result.TOO_EDGE_ON;

            double du = dx;
            double dw = dy * wy + dz * wz;
            double seenWidth = WINDOW_WIDTH * Math.sqrt(Math.max(0, 1 - du * du));
            double seenHeight = WINDOW_HEIGHT * Math.sqrt(Math.max(0, 1 - dw * dw));
            lateralRoom = (seenWidth - BALL_DIAMETER) / 2.0 - APERTURE_MARGIN;
            verticalRoom = (seenHeight - BALL_DIAMETER) / 2.0 - APERTURE_MARGIN;
            if (lateralRoom <= 0 || verticalRoom <= 0) return Result.WINDOW_TOO_NARROW;

            boolean red = alliance == Alliance.RED;
            double otherXMin = red ? BLUE_HIVE_X_MIN : RED_HIVE_X_MIN;
            double otherXMax = red ? BLUE_HIVE_X_MAX : RED_HIVE_X_MAX;
            double r = BALL_DIAMETER / 2.0;
            double prevX = launchX;
            for (int i = 1; i <= FLIGHT_SAMPLES; i++) {
                double t = tf * i / FLIGHT_SAMPLES;
                double f = t / tf;
                double px = launchX + (goal.x() - launchX) * f;
                double py = launchY + (goal.y() - launchY) * f;
                double pz = z0 + vz0 * t - 0.5 * ShooterCalc.gravity() * t * t;

                if (px > otherXMin - r && px < otherXMax + r
                        && py > HIVE_Y_MIN - r && py < HIVE_Y_MAX + r
                        && pz > HIVE_BOTTOM - r && pz < HIVE_TOP + r) {
                    return Result.BLOCKED_OTHER_HIVE;
                }
                if (crossesFramePlate(prevX, px, FRAME_X_MIN, py, pz, r)
                        || crossesFramePlate(prevX, px, FRAME_X_MAX, py, pz, r)) {
                    return Result.BLOCKED_FRAME;
                }
                prevX = px;
            }
        } else {
            lateralRoom = Double.NaN;
            verticalRoom = Double.NaN;
        }

        if (ShooterCalc.hiveMode) {
            lastTurretToleranceDeg = Math.toDegrees(Math.atan2(lateralRoom, Math.max(virtualRange, 1.0)));
            double cosHood = Math.cos(hood);
            double dzPerDv = ShooterCalc.gravity() * virtualRange * virtualRange / (v * v * v * cosHood * cosHood);
            double allowedDz = verticalRoom / Math.max(Math.cos(descent), 0.05);
            double allowedDvMps = (allowedDz / dzPerDv) / 39.37;
            lastFlywheelToleranceTps = allowedDvMps * Math.abs(ShooterCalc.tpsPerBallSpeed(vMps));
        } else {
            lastTurretToleranceDeg = DECODE_TURRET_TOLERANCE_DEG;
            lastFlywheelToleranceTps = DECODE_FLYWHEEL_TOLERANCE_TPS;
        }
        return Result.OK;
    }

    /**
     * Aim-table mode (shot map or tuned points). The table already checked the window, the HIVE and the frame in
     * the simulation, and the geometric checks above assume the real window height, which the table's virtual
     * score height is not. So this only asks the table whether the spot scores, and takes the flywheel tolerance
     * from the map's exit-speed window (tuned points: TunedShotTable.TUNED_SPEED_WINDOW_MPS).
     * A tuned point was shot for real, so the map-only checks (speed window, other HIVE, side-on window) are
     * skipped for it.
     */
    private static Result evaluateTable(boolean turretInRange) {
        double v = ShooterCalc.lastLaunchSpeed;
        double tf = ShooterCalc.lastFlightTime;
        if (!ShooterCalc.lastSolveOk || v <= 0 || tf <= 0) return Result.NO_SOLUTION;
        if (ShooterCalc.requiredTPS > MAX_FLYWHEEL_TPS) return Result.FLYWHEEL_LIMIT;
        if (!turretInRange) return Result.TURRET_LIMIT;

        HiveAimTable.Shot shot = ShooterCalc.tableShot();
        boolean tuned = ShooterCalc.tunedShotInUse();
        if (ShooterCalc.needsTunedPoint()) return Result.NO_TUNED_POINT;
        if (!shot.valid) return Result.NOT_ON_MAP;
        if (!tuned) {
            if (MIN_SPEED_WINDOW_MPS > 0 && shot.halfWindowMps <= MIN_SPEED_WINDOW_MPS + 1e-9) return Result.TOO_TIGHT;
            if (shot.dependsOnOtherHive && !ALLOW_OTHER_HIVE_DEPENDENT) return Result.BLOCKED_OTHER_HIVE;
        }

        // Lateral room: the window looks narrower from the side (it spans x, faces +/- y).
        double bearing = ShooterCalc.lastProjectedBearing;
        double seenWidth = WINDOW_WIDTH * Math.max(Math.abs(Math.sin(bearing)), 0.3);
        double lateralRoom = (seenWidth - BALL_DIAMETER) / 2.0 - APERTURE_MARGIN;
        if (lateralRoom <= 0) {
            if (!tuned) return Result.WINDOW_TOO_NARROW;
            lateralRoom = APERTURE_MARGIN;   // proven on the field: keep a small turret tolerance
        }
        lastTurretToleranceDeg = Math.toDegrees(Math.atan2(lateralRoom, Math.max(ShooterCalc.lastVirtualRange, 1.0)));
        lastFlywheelToleranceTps = TABLE_SPEED_WINDOW_USE * shot.halfWindowMps
                * Math.abs(ShooterCalc.tpsPerBallSpeed(v / 39.37));
        return Result.OK;
    }

    private static boolean crossesFramePlate(double x0, double x1, double plateX, double y, double z, double r) {
        if ((x0 - plateX) * (x1 - plateX) > 0) return false;
        double dyFromCenter = Math.abs(y - FRAME_CENTER_Y);
        if (dyFromCenter >= FRAME_HALF_DEPTH + r) return false;
        double plateTop = FRAME_APEX_HEIGHT * Math.max(0.0, 1.0 - dyFromCenter / FRAME_HALF_DEPTH);
        return z - r < plateTop;
    }
}
