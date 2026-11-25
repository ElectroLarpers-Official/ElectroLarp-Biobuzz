package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Shot log: one CSV row per loop while the robot is shooting, plus the PRE_TRIGGER_SEC before each trigger pull,
 * for working out afterwards why a shot missed. Off unless SHOT_LOG is on.
 *
 * <p>Rows are kept in memory (no file work in the loop) and written when the OpMode stops, to a new file each
 * run: FIRST/shotLog_(date)_(time).csv on the Control Hub. Robot fills a row with row() and hands it over with
 * commit().
 */
@Configurable
public final class ShotLogger {
    /** The shot log is on. Off = nothing is recorded and no file is written. */
    public static boolean SHOT_LOG = true;
    /** How much of the run-up before each trigger pull is kept, seconds (at most PRE_ROWS loops). */
    public static double PRE_TRIGGER_SEC = 0.5;

    public static final String HEADER = "t,loopMs,x,y,headingDeg,vx,vy,speed,omega,range,virtualRange,flightTime,"
            + "turretCmdDeg,rpmTarget,rpmActual,metaMode,shooting,stickBlendDeg";
    public static final int COLS = 18;
    /** Column numbers for row(). t is seconds (System.nanoTime), x / y / range in inches, omega in rad/s. */
    public static final int T = 0, LOOP_MS = 1, X = 2, Y = 3, HEADING_DEG = 4, VX = 5, VY = 6, SPEED = 7, OMEGA = 8,
            RANGE = 9, VIRTUAL_RANGE = 10, FLIGHT_TIME = 11, TURRET_CMD_DEG = 12, RPM_TARGET = 13, RPM_ACTUAL = 14,
            META_MODE = 15, SHOOTING = 16, STICK_BLEND_DEG = 17;

    /** Most rows kept per run (about 5 minutes of shooting at 15 ms loops). Rows after that are dropped. */
    private static final int MAX_ROWS = 20000;
    /** Loops kept for the run-up. */
    private static final int PRE_ROWS = 64;

    private final double[] scratch = new double[COLS];
    private double[] rows;                 // made the first time SHOT_LOG is on
    private int rowCount = 0;
    private final double[] pre = new double[PRE_ROWS * COLS];
    private int preHead = 0, preCount = 0;
    private boolean wasShooting = false;

    /** The row to fill for this loop (reused every loop). */
    public double[] row() { return scratch; }

    /** Records the row filled with row(). Call every loop while SHOT_LOG is on. */
    public void commit(boolean shooting) {
        if (!SHOT_LOG) {
            wasShooting = false;
            preCount = 0;
            return;
        }
        if (shooting) {
            if (!wasShooting) flushRunUp(scratch[T]);
            append(scratch, 0);
        } else {
            System.arraycopy(scratch, 0, pre, preHead * COLS, COLS);
            preHead = (preHead + 1) % PRE_ROWS;
            if (preCount < PRE_ROWS) preCount++;
        }
        wasShooting = shooting;
    }

    /** Trigger pulled: move the run-up rows from the last PRE_TRIGGER_SEC into the log, oldest first. */
    private void flushRunUp(double now) {
        for (int i = preCount; i >= 1; i--) {
            int idx = ((preHead - i) % PRE_ROWS + PRE_ROWS) % PRE_ROWS;
            if (now - pre[idx * COLS + T] <= PRE_TRIGGER_SEC) append(pre, idx * COLS);
        }
        preCount = 0;
    }

    private void append(double[] from, int offset) {
        if (rows == null) rows = new double[MAX_ROWS * COLS];
        if (rowCount >= MAX_ROWS) return;
        System.arraycopy(from, offset, rows, rowCount * COLS, COLS);
        rowCount++;
    }

    public int rowCount() { return rowCount; }

    /**
     * OpMode stop: writes the rows to a new file on a background thread (so stop() returns at once) and empties the
     * log. Returns the file, or null if there was nothing to write.
     */
    public File save() {
        if (rowCount == 0 || rows == null) return null;
        final double[] data = rows;
        final int count = rowCount;
        rows = null;
        rowCount = 0;
        preCount = 0;
        wasShooting = false;

        final File file = newFile();
        Thread writer = new Thread(new Runnable() {
            @Override public void run() {
                try {
                    write(file, data, count);
                } catch (IOException e) {
                    // Nothing useful to do from here: the log is lost, the robot carries on.
                }
            }
        }, "ShotLogSave");
        writer.setDaemon(true);
        writer.start();
        return file;
    }

    private static File newFile() {
        File dir = AppUtil.FIRST_FOLDER;
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File file = new File(dir, "shotLog_" + stamp + ".csv");
        for (int n = 2; file.exists(); n++) file = new File(dir, "shotLog_" + stamp + "_" + n + ".csv");
        return file;
    }

    static void write(File file, double[] data, int count) throws IOException {
        File dir = file.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();
        StringBuilder line = new StringBuilder(256);
        try (BufferedWriter out = new BufferedWriter(new FileWriter(file))) {
            out.write(HEADER);
            out.newLine();
            for (int r = 0; r < count; r++) {
                line.setLength(0);
                for (int c = 0; c < COLS; c++) {
                    if (c > 0) line.append(',');
                    double v = data[r * COLS + c];
                    // 4 decimals for the time, 3 for everything else; whole numbers print without ".0".
                    double scale = c == T ? 1e4 : 1e3;
                    double rounded = Math.round(v * scale) / scale;
                    if (Double.isNaN(v) || Double.isInfinite(v)) line.append(v);
                    else if (rounded == Math.rint(rounded) && Math.abs(rounded) < 1e15) line.append((long) rounded);
                    else line.append(rounded);
                }
                out.write(line.toString());
                out.newLine();
            }
        }
    }
}
