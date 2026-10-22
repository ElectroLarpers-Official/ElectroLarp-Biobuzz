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
// TODO: pick one shooter logic version (control.shotcheck or control.pasted) and delete the other; until then these imports must match Robot's.
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShotCheck;
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

    Alliance a;
    Robot robot;
    GamepadEx gp1;
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
        if (!robot.turret.isConnected()) s.append("turret ");
        if (!robot.latch.isConnected()) s.append("latch ");
        return s.length() == 0 ? "none" : s.toString().trim();
    }

    @Override
    public void execute() {
        // TODO: add driver 2 controls (Shooter.partyMode(), HOOD_STEP hood nudge, ShooterCalc.verticalShiftStep) and manual
        //  fallbacks (turret/hood/flywheel) for when auto-aim or localization fails.
        gp1.update();

        if (gp1.left_trigger_button.isRisingEdge()) {
            robot.intake.intake();
        }
        if (gp1.left_trigger_button.isFallingEdge()) {
            robot.intake.stop();
        }
        // Right trigger held = shoot, but only while the shot can go in. If it stops being possible mid-burst
        // (robot moved), feeding stops; it resumes on its own once the shot is possible again.
        boolean canShoot = !REQUIRE_SHOT_ALLOWED || robot.shotAllowed();
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

    @Override
    public void end() {
        if (robot == null) return;
        OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Shooter", "%.0f / %.0f motor RPM (%s, %s, power %.2f)",
                robot.shooter.rpm(), robot.shooter.targetRpm(),
                robot.shooter.atTarget() ? "at speed" : robot.shooter.isSpinning() ? "spinning up" : "off",
                robot.shooter.mode(), robot.shooter.power());
        telemetry.addData("Range now / projected / virtual", "%.1f / %.1f / %.1f",
                ShooterCalc.lastRange, ShooterCalc.lastProjectedRange, ShooterCalc.lastVirtualRange);
        telemetry.addData("Aim", !ShooterCalc.hiveMode ? "DECODE goal"
                : (ShooterCalc.hiveAimFar() ? "HIVE far cell" : "HIVE audience cell")
                + (robot.underHive() ? " (UNDER HIVE: fixed RPM / hood)" : ""));
        telemetry.addData("Aim table", !ShooterCalc.aimTableInUse() ? "off (fixed score height / entry)"
                : String.format("launch %.1f deg, %.2f m/s +/-%.1f%%%s%s",
                ShooterCalc.lastTableLaunchDeg, ShooterCalc.lastTableSpeedMps, ShooterCalc.lastTableWindowPct,
                ShooterCalc.lastTableValid ? "" : ", NO SHOT HERE",
                ShooterCalc.lastTableDependsOnOtherHive ? ", needs other HIVE tilted right" : ""));
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
        panels.addData("Range in", ShooterCalc.lastRange);
        panels.addData("Virtual range in", ShooterCalc.lastVirtualRange);
        panels.addData("Turret angle deg", ShooterCalc.lastHeadingAngle);
        panels.addData("Robot speed in/s (proj)", ShooterCalc.lastProjectedSpeed);
        panels.addData("Accel in/s^2 (filt)", ShooterCalc.lastAlongTrackAccel);
        panels.addData("Loop ms", robot.loopMs());
        panels.update();   // Panels only; the Driver Station lines above are sent by telemetry.update()
    }
}
