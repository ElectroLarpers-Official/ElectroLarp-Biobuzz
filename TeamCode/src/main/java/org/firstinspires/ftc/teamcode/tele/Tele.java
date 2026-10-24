package org.firstinspires.ftc.teamcode.tele;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.PulseRuntime;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShotCheck;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

@Configurable
public class Tele extends CommandOpMode {
    /**
     * Rotates the field-centric frame so "stick forward" points away from the driver.
     * TODO: verify on the field which way each alliance's driver faces in Pedro coordinates.
     */
    public static double BLUE_FORWARD_OFFSET_DEG = 0;
    public static double RED_FORWARD_OFFSET_DEG = 180;

    /** Manual hood nudge per press. Reserved for driver 2 controls, which are not added yet. */
    public static double HOOD_STEP = 0.05;

    /** Start pose when no saved pose is used, Pedro inches / degrees. */
    public static double START_X = 72, START_Y = 72, START_HEADING_DEG = 0;
    /** Start from where the last auto / TeleOp ended instead of START_X/Y. Turn on once autos are running. */
    public static boolean USE_SAVED_POSE = false;

    /** Only shoot when ShotCheck says the shot can go in. Turn off to test shooting from anywhere. */
    public static boolean REQUIRE_SHOT_ALLOWED = true;

    /** Rumble length while trying to shoot a shot that can't go in, ms. */
    public static int CANT_SHOOT_RUMBLE_MS = 250;

    /** Driver 2 trim steps: motor RPM, hood servo position, turret degrees. */
    public static double RPM_TRIM_STEP = 25, HOOD_TRIM_STEP = 0.005, TURRET_TRIM_STEP_DEG = 1;
    /** Driver 2 right stick dead band in manual mode. */
    public static double MANUAL_STICK_DEADBAND = 0.05;

    Alliance a;
    Robot robot;
    GamepadEx gp1;
    GamepadEx gp2;
    long lastExecNanos = 0;
    Command shootOne;
    TelemetryManager panels;

    public Tele(Alliance a) {
        this.a = a;
    }

    @Override
    public void initialize() {
        // Build the Pulse follower and attach it to PulseRuntime BEFORE constructing Robot,
        // because Robot reads it with PulseRuntime.follower(). This OpMode isn't a PulseOpMode,
        // so nothing attaches the follower for us — we do it here, the one thing PulseOpMode.init
        // would otherwise do. (configurePedro is a PulseOpMode instance method and can't be used here.)
        PulseRuntime.attach((PulseFollower) Constants.create(hardwareMap));
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(startPose());
        robot.follower.update();
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        gp2.left_trigger_button(FloatSupplier::isPress);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        shootOne = robot.shootOne();
        gp1.left_trigger_button(FloatSupplier::isPress);
        gp1.right_trigger_button(FloatSupplier::isPress);
        schedule(infinite(() -> {
            robot.update();
            robot.drive(
                    -gp1.left_stick_y.getVal(),
                    -gp1.left_stick_x.getVal(),
                    -gp1.right_stick_x.getVal(),
                    Math.toRadians(a == Alliance.BLUE ? BLUE_FORWARD_OFFSET_DEG : RED_FORWARD_OFFSET_DEG));
            updateTelemetry();
        }));
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Pose source", usingSavedPose() ? "auto / last run" : "START_X/Y");
        telemetry.addData("Not connected (skipped)", notConnected());
        telemetry.addLine("Ready");
        telemetry.update();
    }

    private boolean usingSavedPose() { return USE_SAVED_POSE && OpModeStorage.endPose != null; }

    private Pose startPose() {
        return usingSavedPose() ? OpModeStorage.endPose : new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private String notConnected() {
        StringBuilder s = new StringBuilder();
        if (!Constants.drivetrainConnected) s.append("drive motors ");
        for (String name : robot.turret.missing()) s.append(name).append(' ');
        if (!robot.latch.isConnected()) s.append("latch ");
        return s.length() == 0 ? "none" : s.toString().trim();
    }

    @Override
    public void execute() {
        gp1.update();
        gp2.update();
        long now = System.nanoTime();
        double dt = lastExecNanos == 0 ? 0.0 : Math.min(0.1, (now - lastExecNanos) / 1e9);
        lastExecNanos = now;

        // Driver 1 X: pose reset. Robot squared in the reset corner (Robot.RESET_X / Y / HEADING_DEG).
        if (gp1.x.isRisingEdge()) {
            robot.resetPose();
            gamepad1.rumble(200);
        }

        driver2(dt);

        if (gp1.left_trigger_button.isRisingEdge()) {
            robot.intake.intake();
        }
        if (gp1.left_trigger_button.isFallingEdge()) {
            robot.intake.stop();
        }
        // Right trigger held = shoot, but only while the shot can go in. If it stops being possible mid-burst
        // (robot moved), feeding stops; it resumes on its own once the shot is possible again.
        boolean canShoot = !REQUIRE_SHOT_ALLOWED || shotOverride() || robot.isManual() || robot.shotAllowed();
        if (gp1.right_trigger_button.isTrue() && canShoot) {
            if (!robot.isShooting()) robot.shootAll();
        } else if (robot.isShooting()) {
            robot.stopShootAll();
        }
        if (gp1.left_bumper.isRisingEdge()) {
            robot.outtake();
        }
        if (gp1.left_bumper.isFallingEdge()) {
            robot.stopOuttake();
        }
        if (gp1.a.isRisingEdge() && canShoot && !Scheduler.isScheduled(shootOne)) {
            schedule(shootOne);
        }
        boolean tryingToShoot = gp1.right_trigger_button.isTrue() || gp1.a.isTrue();
        if (tryingToShoot && !canShoot && !gamepad1.isRumbling()) {
            gamepad1.rumble(CANT_SHOOT_RUMBLE_MS);
        }

    }

    /** Driver 2 holding the left trigger: shoot even where the shot check says no. */
    private boolean shotOverride() { return gp2.left_trigger_button.isTrue(); }

    /**
     * Driver 2: LT hold = shot-check override; d-pad up / down = RPM trim; d-pad right / left = hood trim
     * (steeper / flatter); RB / LB = turret offset for this alliance (+ = counter-clockwise); BACK = all trims to 0;
     * START = manual mode on / off. In manual mode: right stick X turns the turret, A / B / Y = close / mid / far preset.
     */
    private void driver2(double dt) {
        if (gp2.dpad_up.isRisingEdge()) Robot.LIVE_RPM_TRIM += RPM_TRIM_STEP;
        if (gp2.dpad_down.isRisingEdge()) Robot.LIVE_RPM_TRIM -= RPM_TRIM_STEP;
        if (gp2.dpad_right.isRisingEdge()) Robot.LIVE_HOOD_TRIM += HOOD_TRIM_STEP;
        if (gp2.dpad_left.isRisingEdge()) Robot.LIVE_HOOD_TRIM -= HOOD_TRIM_STEP;
        if (gp2.right_bumper.isRisingEdge()) setTurretOffset(turretOffset() + TURRET_TRIM_STEP_DEG);
        if (gp2.left_bumper.isRisingEdge()) setTurretOffset(turretOffset() - TURRET_TRIM_STEP_DEG);
        if (gp2.back.isRisingEdge()) {
            Robot.LIVE_RPM_TRIM = 0;
            Robot.LIVE_HOOD_TRIM = 0;
            setTurretOffset(0);
            gamepad2.rumble(200);
        }
        if (gp2.start.isRisingEdge()) {
            robot.setManual(!robot.isManual());
            gamepad2.rumble(robot.isManual() ? 500 : 200);
        }
        if (robot.isManual()) {
            if (gp2.a.isRisingEdge()) robot.setManualShot(Robot.ManualShot.CLOSE);
            if (gp2.b.isRisingEdge()) robot.setManualShot(Robot.ManualShot.MID);
            if (gp2.y.isRisingEdge()) robot.setManualShot(Robot.ManualShot.FAR);
            double stick = gp2.right_stick_x.getVal();
            if (Math.abs(stick) > MANUAL_STICK_DEADBAND) {
                // stick right = turret clockwise (negative angle)
                robot.nudgeManualTurret(-stick * Robot.MANUAL_TURRET_RATE_DEG_S * dt);
            }
        }
    }

    private double turretOffset() { return a == Alliance.RED ? TurretCalc.turretOffset : TurretCalc.turretOffset2; }
    private void setTurretOffset(double deg) {
        if (a == Alliance.RED) TurretCalc.turretOffset = deg; else TurretCalc.turretOffset2 = deg;
    }

    @Override
    public void end() {
        if (robot == null) return;
        OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    /** Where the RPM / hood come from: hand-tuned points (TunedShotTable) or the shot map. */
    public static String shotSourceText() {
        double near = ShooterCalc.lastTunedNearestIn;
        switch (ShooterCalc.lastShotSource) {
            case TUNED:
                if (!ShooterCalc.tableShot().valid)
                    return String.format("none: nearest tuned point %.1f in away (MAP_FALLBACK off)", near);
                return String.format("TUNED (%d point%s, nearest %.1f in)", ShooterCalc.lastTunedPointsUsed,
                        ShooterCalc.lastTunedPointsUsed == 1 ? "" : "s", near);
            case MAP:
                return Double.isNaN(near) ? "MAP (no tuned points for this cell)"
                        : String.format("MAP (nearest tuned point %.1f in away)", near);
            default:
                return "none";
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Driver 2", "%s%s | trims %+.0f RPM, hood %+.3f, turret %+.0f deg",
                robot.isManual() ? String.format("MANUAL %s, turret %.0f deg", robot.manualShot(), robot.manualTurretDeg())
                        : "auto", shotOverride() ? " | SHOT CHECK OVERRIDE" : "",
                Robot.LIVE_RPM_TRIM, Robot.LIVE_HOOD_TRIM, turretOffset());
        telemetry.addData("Shooter", "%.0f / %.0f motor RPM (%s, %s, power %.2f)",
                robot.shooter.rpm(), robot.shooter.targetRpm(),
                robot.shooter.atTarget() ? "at speed" : robot.shooter.isSpinning() ? "spinning up" : "off",
                robot.shooter.mode(), robot.shooter.power());
        telemetry.addData("Range now / projected / virtual", "%.1f / %.1f / %.1f",
                ShooterCalc.lastRange, ShooterCalc.lastProjectedRange, ShooterCalc.lastVirtualRange);
        telemetry.addData("Aim", (ShooterCalc.hiveAimFar() ? "HIVE far cell" : "HIVE audience cell")
                +(robot.underHive() ? " (UNDER HIVE: fixed RPM / hood)" : ""));
        telemetry.addData("Aim table", !ShooterCalc.aimTableInUse() ? "off (fixed score height / entry)"
                : String.format("launch %.1f deg, %.2f m/s +/-%.1f%%%s%s",
                ShooterCalc.lastTableLaunchDeg, ShooterCalc.lastTableSpeedMps, ShooterCalc.lastTableWindowPct,
                ShooterCalc.lastTableValid ? "" : ", NO SHOT HERE",
                ShooterCalc.lastTableDependsOnOtherHive ? ", needs other HIVE tilted right" : ""));
        telemetry.addData("Shot source", shotSourceText());
        telemetry.addData("Score height / entry", "%.1f in / %.1f deg", ShooterCalc.lastAimHeight, ShooterCalc.lastEntryDeg);
        telemetry.addData("Shot", robot.readyToShoot() ? "READY" : ShotCheck.status());
        telemetry.addData("Tolerance turret deg / flywheel tps", "%.2f / %.0f",
                ShotCheck.turretToleranceDeg(), ShotCheck.flywheelToleranceTps());
        telemetry.addData("Turret heading angle", "%.1f", ShooterCalc.lastHeadingAngle);
        telemetry.addData("SOTM active", ShooterCalc.lastSotmActive);
        telemetry.addData("Flight time", "%.3f", ShooterCalc.lastFlightTime);
        telemetry.addData("Speed projected / aAlongV (filt)", "%.1f / %.1f",
                ShooterCalc.lastProjectedSpeed, ShooterCalc.lastAlongTrackAccel);
        telemetry.addData("RevAmped braking active", ShooterCalc.revAmpedActive);
        telemetry.addData("Hood comp burst deg / vel", "%.2f / %.3f",
                ShooterCalc.lastBurstHoodDeltaDeg, ShooterCalc.lastHoodCompTotal);
        telemetry.addData("Solver iters / converged", "%d / %b", ShooterCalc.lastIterations, ShooterCalc.lastConverged);
        telemetry.addData("Hood", "%.2f", robot.hood.position());
        telemetry.addData("Latch", robot.latch.isOpen() ? "open" : "closed");
        telemetry.addData("Intake", robot.intake.getState());
        telemetry.addData("Loop ms", robot.loopMs());
        telemetry.update();

        // Panels (graph tab): numbers only, so each one can be plotted over time.
        double targetRpm = robot.shooter.isSpinning() ? robot.shooter.targetRpm() : 0;
        panels.addData("Motor RPM", robot.shooter.rpm());
        panels.addData("Target RPM", targetRpm);
        panels.addData("RPM error", targetRpm - robot.shooter.rpm());
        panels.addData("Shooter power", robot.shooter.power());
        panels.addData("Battery V", robot.shooter.voltage());
        panels.addData("Feeding", robot.isShooting() ? 1 : 0);
        panels.addData("Ready", robot.readyToShoot() ? 1 : 0);
        panels.addData("Hood pos", robot.hood.position());
        panels.addData("Launch deg (map)", ShooterCalc.lastTableLaunchDeg);
        panels.addData("Exit speed m/s (map)", ShooterCalc.lastTableSpeedMps);
        panels.addData("Shot source (0 none, 1 tuned, 2 map)", ShooterCalc.lastShotSource.ordinal());
        panels.addData("Nearest tuned point in", Double.isNaN(ShooterCalc.lastTunedNearestIn) ? -1 : ShooterCalc.lastTunedNearestIn);
        panels.addData("Range in", ShooterCalc.lastRange);
        panels.addData("Virtual range in", ShooterCalc.lastVirtualRange);
        panels.addData("Turret angle deg", ShooterCalc.lastHeadingAngle);
        panels.addData("Robot speed in/s (proj)", ShooterCalc.lastProjectedSpeed);
        panels.addData("Accel in/s^2 (filt)", ShooterCalc.lastAlongTrackAccel);
        panels.addData("Manual mode", robot.isManual() ? 1 : 0);
        panels.addData("Shot override", shotOverride() ? 1 : 0);
        panels.addData("Live RPM trim", Robot.LIVE_RPM_TRIM);
        panels.addData("Live hood trim", Robot.LIVE_HOOD_TRIM);
        panels.addData("Turret offset deg", turretOffset());
        panels.addData("Loop ms", robot.loopMs());
        panels.update();   // Panels only; the Driver Station lines above are sent by telemetry.update()
    }
}
