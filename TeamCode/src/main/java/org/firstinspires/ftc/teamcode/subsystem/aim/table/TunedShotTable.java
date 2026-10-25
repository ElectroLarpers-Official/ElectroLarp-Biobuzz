package org.firstinspires.ftc.teamcode.subsystem.aim.table;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Hand-tuned shots: field points where the team found an RPM and hood that score. In HIVE mode the solver uses
 * these instead of the simulated shot map wherever a point is close enough (see ShooterCalc.queryAimTable).
 *
 * <p>Each point is a shooter release point (the turret position, Pedro inches), a cell, a motor RPM and a hood
 * servo position. Between points the values are blended from the nearest few, weighted by 1 / distance^power.
 *
 * <p>Points come from two places:
 * <ul>
 *   <li>{@link TunedShotPoints} in code (the committed set),</li>
 *   <li>a file on the Control Hub, {@code FIRST/shotpoints.csv}, written by the ShotPointTuner OpMode. A file point
 *       replaces any code point of the same cell within {@link #REPLACE_RADIUS_IN}.</li>
 * </ul>
 * When a set is good, print it from the tuner (BACK), paste it into TunedShotPoints, commit, and clear the file.
 *
 * <p>Mirroring. The field is symmetric about x = 72 and y = 72, so one cell's points can serve all four.
 * With {@link #ONE_CELL_FOR_ALL} on (default), every point is stored for RED_AUD (red HIVE, audience cell, the one
 * nearest (0, 0)) and the others use it mirrored: red far cell = flipped across y = 72, blue audience cell =
 * flipped across x = 72, blue far cell = both (turned 180 degrees). Saving from any cell stores the point for
 * RED_AUD. With it off, {@link #MIRROR_BLUE} turns red points 180 degrees for blue (red audience <-> blue far,
 * red far <-> blue audience) and the two red cells are tuned separately.
 */
@Configurable
public final class TunedShotTable {
    /** Use tuned points in HIVE mode. Off = shot map only (the old behaviour). */
    public static boolean USE_TUNED_POINTS = true;
    /** Where no tuned point is close enough: true = use the shot map there (needs ShooterCalc.USE_AIM_TABLE),
     *  false = no shot there (the flywheel and hood still follow the nearest tuned point, and driver 2's shot
     *  override can still shoot). Off with the shot map off (2026-10-25). */
    public static boolean MAP_FALLBACK = false;
    /** A spot counts as tuned when a point of that cell is within this distance, inches. */
    public static double MAX_POINT_DIST_IN = 18.0;
    /** Points blended at a spot (the nearest ones within MAX_POINT_DIST_IN). */
    public static int BLEND_POINTS = 4;
    /** Blend weight = 1 / distance^power. Higher = each point dominates its own area more. */
    public static double BLEND_POWER = 2.0;
    /** Closer than this to a point, use that point exactly, inches. */
    public static double SNAP_IN = 0.75;
    /** Added to every tuned RPM / hood position (motor RPM / servo position). For a new batch of balls etc. */
    public static double RPM_TRIM = 0.0;
    public static double HOOD_TRIM = 0.0;
    /** Every cell uses the RED_AUD points, mirrored (see the class comment). */
    public static boolean ONE_CELL_FOR_ALL = true;
    /** Only when ONE_CELL_FOR_ALL is off: blue uses the red points turned 180 degrees. Off = every cell has its own. */
    public static boolean MIRROR_BLUE = true;
    /** Saving a point removes file points of the same cell this close, and a file point hides code points this
     *  close, inches. */
    public static double REPLACE_RADIUS_IN = 3.0;
    /** Exit-speed error a tuned shot is assumed to tolerate, m/s. Only sets the "flywheel at speed" tolerance
     *  ShotCheck.ready uses (0.15 m/s is about 75 motor RPM). */
    public static double TUNED_SPEED_WINDOW_MPS = 0.15;
    /** Flight time = factor * sqrt(2 * (R tan(launch) - h) / g), h = window centre above the release point.
     *  1.03 matches the drag simulation to a few percent from 50 to 78 deg. Only used for shoot-while-moving lead. */
    public static double FLIGHT_TIME_FACTOR = 1.03;
    /** Window centre above the tiles, inches (field CAD, the height the shot map aimed at). */
    public static double WINDOW_CENTRE_HEIGHT_IN = 58.4;

    public static final String FILE_NAME = "shotpoints.csv";
    private static final double FIELD_SIZE_IN = 144.0;

    /** One tuned shot. x, y = shooter release point (turret position), Pedro inches, in the cell's own frame. */
    public static final class Point {
        public final HiveAimTable.Cell cell;
        public final double x, y, rpm, hood;
        public final boolean fromFile;

        public Point(HiveAimTable.Cell cell, double x, double y, double rpm, double hood, boolean fromFile) {
            this.cell = cell; this.x = x; this.y = y; this.rpm = rpm; this.hood = hood; this.fromFile = fromFile;
        }

        /** The line to paste into TunedShotPoints.POINTS. */
        public String javaLine() {
            return String.format(Locale.US, "p(%s, %.1f, %.1f, %.0f, %.3f),", cell.name(), x, y, rpm, hood);
        }

        String csvLine() {
            return String.format(Locale.US, "%s,%.2f,%.2f,%.1f,%.4f", cell.name(), x, y, rpm, hood);
        }
    }

    /** Lookup result. Reused by the caller. */
    public static final class Shot {
        public double rpm, hood;
        /** A point of this cell is within MAX_POINT_DIST_IN. False: rpm / hood are the nearest point's. */
        public boolean valid;
        public double nearestIn;
        public int pointsUsed;
    }

    private static final List<Point> codePoints = new ArrayList<>();
    private static final List<Point> filePoints = new ArrayList<>();
    /** Code points not hidden by a file point, plus all file points. */
    private static final List<Point> active = new ArrayList<>();
    private static boolean loaded = false;
    private static String lastError = null;

    private TunedShotTable() {}

    /** (Re)loads the code points and the file. Call at init; the tuner calls it after editing the file. */
    public static synchronized void load() {
        codePoints.clear();
        codePoints.addAll(TunedShotPoints.points());
        filePoints.clear();
        lastError = null;
        File f = file();
        if (f.exists()) {
            try (BufferedReader r = new BufferedReader(new FileReader(f))) {
                String line;
                int n = 0;
                while ((line = r.readLine()) != null) {
                    n++;
                    line = line.trim();
                    if (line.isEmpty() || line.startsWith("#") || line.startsWith("cell")) continue;
                    String[] s = line.split(",");
                    try {
                        filePoints.add(new Point(HiveAimTable.Cell.valueOf(s[0].trim()),
                                Double.parseDouble(s[1]), Double.parseDouble(s[2]),
                                Double.parseDouble(s[3]), Double.parseDouble(s[4]), true));
                    } catch (RuntimeException e) {
                        lastError = FILE_NAME + " line " + n + " skipped: " + line;
                    }
                }
            } catch (IOException e) {
                lastError = "can't read " + f + ": " + e.getMessage();
            }
        }
        rebuild();
        loaded = true;
    }

    private static void rebuild() {
        active.clear();
        for (Point c : codePoints) {
            boolean hidden = false;
            for (Point p : filePoints) {
                if (p.cell == c.cell && Math.hypot(p.x - c.x, p.y - c.y) <= REPLACE_RADIUS_IN) { hidden = true; break; }
            }
            if (!hidden) active.add(c);
        }
        active.addAll(filePoints);
    }

    private static void ensureLoaded() { if (!loaded) load(); }

    public static File file() { return new File(AppUtil.FIRST_FOLDER, FILE_NAME); }
    public static String lastError() { return lastError; }

    /** The cell whose points serve this cell. */
    private static HiveAimTable.Cell storedCell(HiveAimTable.Cell cell) {
        if (ONE_CELL_FOR_ALL) return HiveAimTable.Cell.RED_AUD;
        if (!MIRROR_BLUE) return cell;
        switch (cell) {
            case BLUE_FAR: return HiveAimTable.Cell.RED_AUD;
            case BLUE_AUD: return HiveAimTable.Cell.RED_FAR;
            default: return cell;
        }
    }
    private static boolean isRed(HiveAimTable.Cell c) { return c == HiveAimTable.Cell.RED_AUD || c == HiveAimTable.Cell.RED_FAR; }
    private static boolean isFar(HiveAimTable.Cell c) { return c == HiveAimTable.Cell.RED_FAR || c == HiveAimTable.Cell.BLUE_FAR; }

    /** A point (x, y) for {@code cell} in the frame of the cell its points are stored for: flipped across x = 72
     *  when the alliance differs, across y = 72 when the side (audience / far) differs. Its own inverse. */
    private static double storedX(HiveAimTable.Cell cell, double x) {
        return isRed(storedCell(cell)) != isRed(cell) ? FIELD_SIZE_IN - x : x;
    }
    private static double storedY(HiveAimTable.Cell cell, double y) {
        return isFar(storedCell(cell)) != isFar(cell) ? FIELD_SIZE_IN - y : y;
    }

    /** Nearest-points buffers for lookup (runs several times a loop: reused, not reallocated). */
    private static Point[] best = new Point[4];
    private static double[] bestD = new double[4];

    /**
     * Tuned shot for a release point (Pedro inches) at a cell. Returns false when the cell has no points at all.
     * Otherwise {@code out.valid} says whether a point is within MAX_POINT_DIST_IN.
     */
    public static synchronized boolean lookup(HiveAimTable.Cell cell, double x, double y, Shot out) {
        ensureLoaded();
        HiveAimTable.Cell sc = storedCell(cell);
        double sx = storedX(cell, x), sy = storedY(cell, y);
        x = sx; y = sy;

        int k = Math.max(1, BLEND_POINTS);
        if (best.length < k) { best = new Point[k]; bestD = new double[k]; }   // only when BLEND_POINTS is raised
        Point[] best = TunedShotTable.best;
        double[] bestD = TunedShotTable.bestD;
        int found = 0;
        for (int n = 0, size = active.size(); n < size; n++) {
            Point p = active.get(n);
            if (p.cell != sc) continue;
            double d = Math.hypot(p.x - x, p.y - y);
            // insertion into the k nearest
            int i = Math.min(found, k - 1);
            if (found == k && d >= bestD[k - 1]) continue;
            while (i > 0 && bestD[i - 1] > d) { best[i] = best[i - 1]; bestD[i] = bestD[i - 1]; i--; }
            best[i] = p; bestD[i] = d;
            if (found < k) found++;
        }
        if (found == 0) return false;

        out.nearestIn = bestD[0];
        out.valid = bestD[0] <= MAX_POINT_DIST_IN;
        if (!out.valid || bestD[0] <= SNAP_IN) {
            out.rpm = best[0].rpm;
            out.hood = best[0].hood;
            out.pointsUsed = 1;
        } else {
            double w = 0, rpm = 0, hood = 0;
            int used = 0;
            for (int i = 0; i < found; i++) {
                if (bestD[i] > MAX_POINT_DIST_IN) break;
                double wi = 1.0 / Math.pow(bestD[i], BLEND_POWER);
                w += wi; rpm += wi * best[i].rpm; hood += wi * best[i].hood;
                used++;
            }
            out.rpm = rpm / w;
            out.hood = hood / w;
            out.pointsUsed = used;
        }
        out.rpm += RPM_TRIM;
        out.hood += HOOD_TRIM;
        return true;
    }

    // ---- Editing (ShotPointTuner) -------------------------------------------------

    /** Saves a point to the file (replacing file points of that cell within REPLACE_RADIUS_IN). x, y = release
     *  point in field coordinates for {@code cell}; RPM / hood as commanded (trims are taken off). */
    public static synchronized Point save(HiveAimTable.Cell cell, double x, double y, double rpm, double hood) throws IOException {
        ensureLoaded();
        HiveAimTable.Cell sc = storedCell(cell);
        double sx = storedX(cell, x), sy = storedY(cell, y);
        x = sx; y = sy;
        Point p = new Point(sc, x, y, rpm - RPM_TRIM, hood - HOOD_TRIM, true);
        for (int i = filePoints.size() - 1; i >= 0; i--) {
            Point o = filePoints.get(i);
            if (o.cell == sc && Math.hypot(o.x - x, o.y - y) <= REPLACE_RADIUS_IN) filePoints.remove(i);
        }
        filePoints.add(p);
        writeFile();
        rebuild();
        return p;
    }

    /** Nearest active point of the cell to a release point, or null. */
    public static synchronized Point nearest(HiveAimTable.Cell cell, double x, double y) {
        ensureLoaded();
        HiveAimTable.Cell sc = storedCell(cell);
        double sx = storedX(cell, x), sy = storedY(cell, y);
        x = sx; y = sy;
        Point best = null;
        double bestD = Double.POSITIVE_INFINITY;
        for (Point p : active) {
            if (p.cell != sc) continue;
            double d = Math.hypot(p.x - x, p.y - y);
            if (d < bestD) { bestD = d; best = p; }
        }
        return best;
    }

    /** Distance from a release point to a stored point, in the cell's frame. */
    public static double distance(HiveAimTable.Cell cell, double x, double y, Point p) {
        return Math.hypot(p.x - storedX(cell, x), p.y - storedY(cell, y));
    }

    /** Removes a file point. Code points can't be removed here (edit TunedShotPoints). */
    public static synchronized boolean delete(Point p) throws IOException {
        if (p == null || !p.fromFile || !filePoints.remove(p)) return false;
        writeFile();
        rebuild();
        return true;
    }

    /** Deletes every file point (after they have been pasted into TunedShotPoints). */
    public static synchronized void clearFile() throws IOException {
        ensureLoaded();
        filePoints.clear();
        writeFile();
        rebuild();
    }

    public static synchronized List<Point> filePoints() { ensureLoaded(); return new ArrayList<>(filePoints); }
    public static synchronized int count(boolean fromFile) {
        ensureLoaded();
        int n = 0;
        for (Point p : active) if (p.fromFile == fromFile) n++;
        return n;
    }

    /** Writes the file points as Java lines to the log (logcat tag "ShotPoints") and returns them. */
    public static synchronized List<String> printJava() {
        List<String> lines = new ArrayList<>();
        for (Point p : filePoints) lines.add(p.javaLine());
        RobotLog.ii("ShotPoints", "---- " + lines.size() + " file points, paste into TunedShotPoints.POINTS ----");
        for (String l : lines) RobotLog.ii("ShotPoints", l);
        return lines;
    }

    private static void writeFile() throws IOException {
        File f = file();
        File dir = f.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();
        try (FileWriter w = new FileWriter(f, false)) {
            w.write("cell,x,y,rpm,hood\n");
            for (Point p : filePoints) w.write(p.csvLine() + "\n");
        }
    }
}
