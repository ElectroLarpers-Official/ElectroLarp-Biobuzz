package org.firstinspires.ftc.teamcode.subsystem.aim.shot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystem.aim.table.HiveAimTable;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public final class ShotCheck {

    public enum Result {
        OK("ok"),
        NO_SOLUTION("no trajectory reaches the aim point"),
        FLYWHEEL_LIMIT("too far: flywheel can't spin that fast"),
        TURRET_LIMIT("turret can't rotate that far"),
        TOO_CLOSE("too close: ball still rising at the opening"),
        BEHIND_WINDOW("behind the cell opening"),
        WINDOW_TOO_NARROW("opening looks narrower than the ball from here"),
        BLOCKED_OTHER_HIVE("other alliance's HIVE is in the way"),
        NOT_ON_MAP("shot map: no scoring shot from here"),
        UNDER_HIVE("under the HIVE: holding fixed RPM / hood, no shot"),
        TOO_TIGHT("shot map: shot too tight (speed window not over the minimum)"),
        NO_TUNED_POINT("no tuned shot point close enough (MAP_FALLBACK is off)");

        public final String text;
        Result(String text) { this.text = text; }
    }

    // ---- Ball and shooter ----------------------------------------------------
    /** Ball diameter, inches. */
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

    // ---- CELL opening (manual 9.6.2) ---------------------------------------------
    public static double WINDOW_WIDTH = 20.0;   // across the field (x)
    public static double WINDOW_HEIGHT = 14.0;  // along the tilted opening

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

    /** Samples along the flight for the other-HIVE check (fallback check only). */
    public static int FLIGHT_SAMPLES = 40;

    // ---- Outputs -----------------------------------------------------------
    public static Result lastResult = Result.NO_SOLUTION;
    /** Turret aim error the opening leaves room for, degrees. */
    public static double lastTurretToleranceDeg = 0.0;
    /** Flywheel speed error the opening leaves room for, ticks/s. */
    public static double lastFlywheelToleranceTps = 0.0;

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
        lastResult = Result.UNDER_HIVE;
    }

    public static boolean check(Alliance alliance, Pose goal) {
        lastTurretToleranceDeg = 0.0;
        lastFlywheelToleranceTps = 0.0;
        lastResult = evaluate(alliance, goal, !TurretCalc.outOfRange());
        return allowed();
    }

    private static Result evaluate(Alliance alliance, Pose goal, boolean turretInRange) {
        if (ShooterCalc.aimTableInUse()) return evaluateTable(turretInRange);
        return evaluateGeometry(alliance, goal, turretInRange);
    }

    /**
     * Fallback check, for a shot from the plain physics solver (fixed hiveAimHeight / hiveEntryDeg). Only runs
     * when this loop's shot came from neither a tuned point nor the shot map: tuned points switched off
     * (USE_TUNED_POINTS false), or a cell with no tuned points at all. It asks only what that solver can answer:
     * can the flywheel and turret do it, is the ball falling and coming from the cell's own side at the window,
     * does the other alliance's HIVE block it, and how much aim / speed error the window leaves room for.
     */
    private static Result evaluateGeometry(Alliance alliance, Pose goal, boolean turretInRange) {
        double v = ShooterCalc.lastLaunchSpeed;
        double hood = ShooterCalc.lastLaunchHoodRad;
        double tf = ShooterCalc.lastFlightTime;
        double range = ShooterCalc.lastProjectedRange;
        double virtualRange = ShooterCalc.lastVirtualRange;

        if (!ShooterCalc.lastSolveOk || v <= 0 || tf <= 0) return Result.NO_SOLUTION;
        if (ShooterCalc.requiredTPS > MAX_FLYWHEEL_TPS) return Result.FLYWHEEL_LIMIT;
        if (!turretInRange) return Result.TURRET_LIMIT;

        double g = ShooterCalc.gravity();
        double vz0 = v * Math.sin(hood);
        double vh = range / tf;
        double vzEnd = vz0 - g * tf;
        if (vzEnd >= 0) return Result.TOO_CLOSE;

        // The cell opens toward its own side of the field (+y for the far cell, -y for the audience cell):
        // the ball has to be travelling toward the HIVE from that side.
        double bearing = ShooterCalc.lastProjectedBearing;
        double side = ShooterCalc.hiveAimFar() ? 1.0 : -1.0;
        if (Math.sin(bearing) * side >= 0) return Result.BEHIND_WINDOW;

        double lateralRoom = lateralRoom(bearing);
        if (lateralRoom <= 0) return Result.WINDOW_TOO_NARROW;

        // Other alliance's HIVE: sample the arc (straight line on the floor, parabola in height).
        boolean red = alliance == Alliance.RED;
        double otherXMin = red ? BLUE_HIVE_X_MIN : RED_HIVE_X_MIN;
        double otherXMax = red ? BLUE_HIVE_X_MAX : RED_HIVE_X_MAX;
        double r = BALL_DIAMETER / 2.0;
        double launchX = goal.x() - range * Math.cos(bearing);
        double launchY = goal.y() - range * Math.sin(bearing);
        for (int i = 1; i <= FLIGHT_SAMPLES; i++) {
            double f = (double) i / FLIGHT_SAMPLES;
            double t = tf * f;
            double px = launchX + (goal.x() - launchX) * f;
            double py = launchY + (goal.y() - launchY) * f;
            double pz = ShooterCalc.launchHeight + vz0 * t - 0.5 * g * t * t;
            if (px > otherXMin - r && px < otherXMax + r
                    && py > HIVE_Y_MIN - r && py < HIVE_Y_MAX + r
                    && pz > HIVE_BOTTOM - r && pz < HIVE_TOP + r) {
                return Result.BLOCKED_OTHER_HIVE;
            }
        }

        // Tolerances: turret from the window's seen width; flywheel from how far a speed error moves the ball
        // up / down at the window vs. the window's height.
        lastTurretToleranceDeg = Math.toDegrees(Math.atan2(lateralRoom, Math.max(virtualRange, 1.0)));
        double verticalRoom = (WINDOW_HEIGHT - BALL_DIAMETER) / 2.0 - APERTURE_MARGIN;
        double cosHood = Math.cos(hood);
        double dzPerDv = g * virtualRange * virtualRange / (v * v * v * cosHood * cosHood);
        double descent = Math.atan2(-vzEnd, vh);
        double allowedDz = Math.max(verticalRoom, 0.0) / Math.max(Math.cos(descent), 0.05);
        lastFlywheelToleranceTps = (allowedDz / dzPerDv) / 39.37 * Math.abs(ShooterCalc.tpsPerBallSpeed());
        return Result.OK;
    }

    /** Room the ball has on each side inside the window, inches. The window spans x and faces +/- y, so it looks
     *  narrower from the side. */
    private static double lateralRoom(double bearing) {
        double seenWidth = WINDOW_WIDTH * Math.max(Math.abs(Math.sin(bearing)), 0.3);
        return (seenWidth - BALL_DIAMETER) / 2.0 - APERTURE_MARGIN;
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

        double lateralRoom = lateralRoom(ShooterCalc.lastProjectedBearing);
        if (lateralRoom <= 0) {
            if (!tuned) return Result.WINDOW_TOO_NARROW;
            lateralRoom = APERTURE_MARGIN;   // proven on the field: keep a small turret tolerance
        }
        lastTurretToleranceDeg = Math.toDegrees(Math.atan2(lateralRoom, Math.max(ShooterCalc.lastVirtualRange, 1.0)));
        lastFlywheelToleranceTps = TABLE_SPEED_WINDOW_USE * shot.halfWindowMps
                * Math.abs(ShooterCalc.tpsPerBallSpeed());
        return Result.OK;
    }
}
