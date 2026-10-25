package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShotCheck;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.table.HiveAimTable;
import org.firstinspires.ftc.teamcode.subsystem.aim.table.TunedShotTable;
import org.firstinspires.ftc.teamcode.tele.Tele;
import org.firstinspires.ftc.teamcode.util.TeleTelemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

import java.io.IOException;
import java.util.List;
import java.util.Locale;

/**
 * Tunes shot points (TunedShotTable) on the field. Drive (or push) the robot to a spot, adjust RPM and hood until
 * the shots go in, press A to save. Points go to FIRST/shotpoints.csv on the Hub and are used by TeleOp right away
 * (TeleOp loads the file at init).
 *
 * <p>Pose: Pinpoint, starting from Tele.START_X / START_Y / START_HEADING_DEG. The turret aims at the cell on the
 * robot's side of the field (if plugged in). RPM / hood start at what TeleOp would shoot here (tuned points, else
 * the shot map) and follow it as you drive, until you touch the d-pad or bumpers. Shooting is never blocked here.
 *
 * <p>Gamepad 1 - init: right bumper = alliance. Running:
 * <ul>
 *   <li>sticks: drive (if the drive motors are plugged in)</li>
 *   <li>right trigger: shoot (hold), left trigger: intake (hold), Y: flywheel on/off</li>
 *   <li>d-pad up / down: RPM +/- RPM_STEP, right / left bumper: RPM +/- RPM_BIG_STEP</li>
 *   <li>d-pad right / left: hood +/- HOOD_STEP (servo position; up = steeper)</li>
 *   <li>START: back to the predicted RPM / hood (follow mode)</li>
 *   <li>A: save a point here, B: delete the nearest file point, X: reload the file</li>
 *   <li>BACK: print all file points as Java lines (Driver Station + logcat tag ShotPoints);
 *       hold BACK for CLEAR_HOLD_S: delete all file points (do this after pasting them into TunedShotPoints)</li>
 * </ul>
 */
@Configurable
@TeleOp(group = "Shooter Test")
public class ShotPointTuner extends CommandOpMode {
    public static double RPM_STEP = 25;
    public static double RPM_BIG_STEP = 100;
    public static double HOOD_STEP = 0.005;
    public static boolean RED_ALLIANCE = true;
    public static double CLEAR_HOLD_S = 3.0;
    /** Java lines shown on the Driver Station after BACK, at most. All of them always go to logcat. */
    public static int MAX_PRINT_LINES = 30;

    Robot robot;
    Alliance alliance;
    GamepadEx gp1;
    TelemetryManager panels;
    List<LynxModule> hubs;

    boolean flywheelOn = true;
    /** True: RPM / hood follow the prediction. False: they hold what the driver set. */
    boolean follow = true;
    double tuneRpm = 0, tuneHood = 0;
    double predRpm = 0, predHood = 0;
    HiveAimTable.Cell cell = null;
    Pose releasePoint = new Pose(0, 0, 0);
    String message = "";
    List<String> printed = null;
    long backDownNanos = 0;
    boolean cleared = false;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        gp1.left_trigger_button(FloatSupplier::isPress);
        gp1.right_trigger_button(FloatSupplier::isPress);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        alliance = RED_ALLIANCE ? Alliance.RED : Alliance.BLUE;
        robot = new Robot(hardwareMap, alliance);   // also loads the shot map and the tuned points
        robot.follower.setPose(new Pose(Tele.START_X, Tele.START_Y, Math.toRadians(Tele.START_HEADING_DEG)));
        robot.follower.update();
        hubs = hardwareMap.getAll(LynxModule.class);
        if (TunedShotTable.lastError() != null) message = TunedShotTable.lastError();
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        if (gp1.right_bumper.isRisingEdge()) RED_ALLIANCE = !RED_ALLIANCE;
        telemetry.addData("Alliance (RB to change)", RED_ALLIANCE ? "RED" : "BLUE");
        telemetry.addData("Start pose", "(%.1f, %.1f) %.0f deg (Tele.START_X / Y)", Tele.START_X, Tele.START_Y,
                Tele.START_HEADING_DEG);
        telemetry.addData("Points", "%d in code, %d in %s", TunedShotTable.count(false), TunedShotTable.count(true),
                TunedShotTable.file());
        if (!message.isEmpty()) telemetry.addLine(message);
        telemetry.update();
    }

    @Override
    public void onStart() {
        // The alliance may have changed during init: Robot keeps the one it was built with, aiming uses this one.
        alliance = RED_ALLIANCE ? Alliance.RED : Alliance.BLUE;
    }

    @Override
    public void execute() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        gp1.update();
        robot.follower.update();

        robot.drive(-gp1.left_stick_y.getVal(), -gp1.left_stick_x.getVal(), -gp1.right_stick_x.getVal(),
                Math.toRadians(alliance == Alliance.BLUE ? Tele.BLUE_FORWARD_OFFSET_DEG : Tele.RED_FORWARD_OFFSET_DEG));

        aim();
        handleButtons();

        if (flywheelOn && tuneRpm > 0) robot.shooter.setTargetRpm(tuneRpm); else robot.shooter.stop();
        robot.hood.setPosition(tuneHood);
        robot.shooter.update();

        showTelemetry();
    }

    /** Same solve as TeleOp for the robot standing still here: turret, prediction, cell, release point. */
    private void aim() {
        Pose pose = robot.follower.pose();
        Pose turretPose = Turret.getTurretPose(pose);
        Pose goal = ShooterCalc.goal(alliance, pose.y());
        Vector2D turretToGoal = Turret.getTurretToGoalVector(turretPose, goal);
        double[] result = ShooterCalc.solve(pose.heading(), turretToGoal, Vector2D.zero(), Vector2D.zero(), 0.0);
        ShotCheck.check(alliance, goal);
        robot.turret.setPositionAll(TurretCalc.solve(result[2], 0.0, alliance)); // TODO delete: -> setPosition

        cell = HiveAimTable.Cell.of(alliance == Alliance.RED, ShooterCalc.hiveAimFar());
        releasePoint = turretPose;
        predRpm = ShooterCalc.requiredRPM;
        predHood = result[1];
        if (follow) {
            tuneRpm = predRpm;
            tuneHood = predHood;
        }
    }

    private void handleButtons() {
        if (gp1.right_trigger_button.isRisingEdge()) robot.startShooting();
        if (gp1.right_trigger_button.isFallingEdge()) robot.stopShooting();
        if (!robot.isShooting()) {
            if (gp1.left_trigger_button.isRisingEdge()) robot.intake.on();
            if (gp1.left_trigger_button.isFallingEdge()) robot.intake.off();
        }
        if (gp1.y.isRisingEdge()) flywheelOn = !flywheelOn;

        if (gp1.dpad_up.isRisingEdge()) nudge(RPM_STEP, 0);
        if (gp1.dpad_down.isRisingEdge()) nudge(-RPM_STEP, 0);
        if (gp1.right_bumper.isRisingEdge()) nudge(RPM_BIG_STEP, 0);
        if (gp1.left_bumper.isRisingEdge()) nudge(-RPM_BIG_STEP, 0);
        if (gp1.dpad_right.isRisingEdge()) nudge(0, HOOD_STEP);
        if (gp1.dpad_left.isRisingEdge()) nudge(0, -HOOD_STEP);
        if (gp1.start.isRisingEdge()) { follow = true; message = "Following the prediction"; }

        if (gp1.a.isRisingEdge()) save();
        if (gp1.b.isRisingEdge()) deleteNearest();
        if (gp1.x.isRisingEdge()) {
            TunedShotTable.load();
            message = TunedShotTable.lastError() != null ? TunedShotTable.lastError()
                    : String.format(Locale.US, "Reloaded: %d code + %d file points", TunedShotTable.count(false),
                    TunedShotTable.count(true));
        }

        // BACK: print; hold: clear the file.
        if (gp1.back.isRisingEdge()) {
            printed = TunedShotTable.printJava();
            message = printed.size() + " file points printed (logcat tag ShotPoints). Hold BACK "
                    + (int) CLEAR_HOLD_S + " s to clear the file.";
            backDownNanos = System.nanoTime();
            cleared = false;
        }
        if (gp1.back.isTrue() && backDownNanos != 0 && !cleared
                && (System.nanoTime() - backDownNanos) / 1e9 >= CLEAR_HOLD_S) {
            try {
                TunedShotTable.clearFile();
                message = "File cleared. Code points are still used.";
                printed = null;
            } catch (IOException e) {
                message = "Couldn't clear the file: " + e.getMessage();
            }
            cleared = true;
        }
        if (!gp1.back.isTrue()) backDownNanos = 0;
    }

    private void nudge(double rpm, double hood) {
        follow = false;
        tuneRpm = Math.max(0, tuneRpm + rpm);
        tuneHood = Math.max(0, Math.min(1, tuneHood + hood));
    }

    private void save() {
        if (cell == null) { message = "Not aiming at a HIVE cell"; return; }
        if (ShooterConstants.underHive(robot.follower.pose().x(), robot.follower.pose().y())) {
            message = "Under the HIVE: TeleOp holds a fixed RPM / hood here, not saved";
            return;
        }
        try {
            TunedShotTable.Point p = TunedShotTable.save(cell, releasePoint.x(), releasePoint.y(), tuneRpm,
                    robot.hood.position());
            message = "Saved " + p.javaLine();
            follow = true;   // the prediction now includes this point
        } catch (IOException e) {
            message = "Couldn't save: " + e.getMessage();
        }
    }

    private void deleteNearest() {
        if (cell == null) return;
        TunedShotTable.Point p = TunedShotTable.nearest(cell, releasePoint.x(), releasePoint.y());
        if (p == null) { message = "No points for this cell"; return; }
        double d = TunedShotTable.distance(cell, releasePoint.x(), releasePoint.y(), p);
        if (!p.fromFile) {
            message = String.format(Locale.US, "Nearest point (%.1f in) is in code: remove it in TunedShotPoints, "
                    + "or save a new one within %.0f in to replace it", d, TunedShotTable.REPLACE_RADIUS_IN);
            return;
        }
        try {
            TunedShotTable.delete(p);
            message = String.format(Locale.US, "Deleted %s (%.1f in away)", p.javaLine(), d);
        } catch (IOException e) {
            message = "Couldn't delete: " + e.getMessage();
        }
    }

    private void showTelemetry() {
        Pose pose = robot.follower.pose();
        TunedShotTable.Point near = cell == null ? null : TunedShotTable.nearest(cell, releasePoint.x(), releasePoint.y());
        double nearD = near == null ? Double.NaN : TunedShotTable.distance(cell, releasePoint.x(), releasePoint.y(), near);

        telemetry.addData("Alliance / cell", "%s / %s", alliance, cell);
        telemetry.addData("Pose", "(%.1f, %.1f) %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Release point (saved)", "(%.1f, %.1f), %.1f in from the cell", releasePoint.x(),
                releasePoint.y(), ShooterCalc.lastRange);
        telemetry.addData("RPM", "%.0f set / %.0f actual%s", tuneRpm, robot.shooter.rpm(), flywheelOn ? "" : " (OFF, Y)");
        telemetry.addData("Hood", "%.3f (%.1f deg launch)", robot.hood.position(),
                ShooterCalc.launchDegForHoodPosition(robot.hood.position()));
        telemetry.addData("Mode", follow ? "FOLLOW prediction" : "MANUAL (START = follow)");
        telemetry.addData("Prediction", "%.0f RPM, hood %.3f, from %s", predRpm, predHood, TeleTelemetry.shotSourceText());
        telemetry.addData("TeleOp would", ShotCheck.allowed() ? "shoot here" : "not shoot: " + ShotCheck.status());
        telemetry.addData("Nearest point", near == null ? "none for this cell"
                : String.format(Locale.US, "%.1f in: %.0f RPM, hood %.3f (%s)", nearD, near.rpm, near.hood,
                near.fromFile ? "file" : "code"));
        telemetry.addData("Points", "%d code, %d file", TunedShotTable.count(false), TunedShotTable.count(true));
        if (!message.isEmpty()) telemetry.addLine(message);
        if (printed != null) {
            telemetry.addLine("---- paste into TunedShotPoints.POINTS ----");
            for (int i = 0; i < printed.size() && i < MAX_PRINT_LINES; i++) telemetry.addLine(printed.get(i));
            if (printed.size() > MAX_PRINT_LINES) telemetry.addLine("... rest in logcat (tag ShotPoints)");
        }
        telemetry.update();

        panels.addData("Motor RPM", robot.shooter.rpm());
        panels.addData("Target RPM", flywheelOn ? tuneRpm : 0);
        panels.addData("Hood pos", robot.hood.position());
        panels.addData("Shooting", robot.isShooting() ? 1 : 0);
        panels.update();
    }

    @Override
    public void end() {
        if (robot != null) robot.stop();
    }
}
