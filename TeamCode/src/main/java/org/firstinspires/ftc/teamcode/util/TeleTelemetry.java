package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;

/** TeleOp telemetry: Driver Station lines plus Panels graphs. Call {@link #update} once per loop. */
public class TeleTelemetry {
    private final Telemetry telemetry;
    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Robot robot;
    private final Alliance a;

    public TeleTelemetry(Telemetry telemetry, Robot robot, Alliance a) {
        this.telemetry = telemetry;
        this.robot = robot;
        this.a = a;
    }

    public void update(double turretOffset) {
        driverStation(turretOffset);
        panels(turretOffset);
    }

    /** Which tuned points the RPM / hood came from (TunedShotTable). */
    public static String shotSourceText() {
        if (Double.isNaN(ShooterCalc.lastTunedNearestIn)) return "NO TUNED POINTS for this cell (flywheel off)";
        return String.format("%d point%s, nearest %.1f in", ShooterCalc.lastTunedPointsUsed,
                ShooterCalc.lastTunedPointsUsed == 1 ? "" : "s", ShooterCalc.lastTunedNearestIn);
    }

    private String driver2Text(double turretOffset) {
        String mode = robot.isManual()
                ? String.format("MANUAL, turret %.0f deg", robot.manualTurretDeg())
                : "auto";
        return String.format("%s%s | trims %+.0f RPM, hood %+.3f, turret %+.0f deg", mode,
                robot.isTurretForward() ? " | TURRET FORWARD" : "",
                Robot.LIVE_RPM_TRIM, Robot.LIVE_HOOD_TRIM, turretOffset);
    }

    private void driverStation(double turretOffset) {
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Driver 2", driver2Text(turretOffset));
        telemetry.addData("Shooter", "%.0f / %.0f motor RPM (%s, %s, power %.2f)",
                robot.shooter.rpm(), robot.shooter.targetRpm(),
                robot.shooter.atTarget() ? "at speed" : robot.shooter.isSpinning() ? "spinning up" : "off",
                robot.shooter.mode(), robot.shooter.power());
        telemetry.addData("Range now / projected / virtual", "%.1f / %.1f / %.1f",
                ShooterCalc.lastRange, ShooterCalc.lastProjectedRange, ShooterCalc.lastVirtualRange);
        telemetry.addData("Aim", (ShooterCalc.hiveAimFar() ? "HIVE far cell" : "HIVE audience cell")
                + (robot.underHive() ? " (UNDER HIVE: fixed RPM / hood)" : ""));
        telemetry.addData("Shot source", shotSourceText());
        telemetry.addData("Turret heading angle", "%.1f", ShooterCalc.lastHeadingAngle);
        telemetry.addData("SOTM active", ShooterCalc.lastSotmActive);
        telemetry.addData("Flight time", "%.3f", ShooterCalc.lastFlightTime);
        telemetry.addData("Speed projected / aAlongV (filt)", "%.1f / %.1f",
                ShooterCalc.lastProjectedSpeed, ShooterCalc.lastAlongTrackAccel);
        telemetry.addData("RevAmped braking active", ShooterCalc.revAmpedActive);
        telemetry.addData("Hood burst comp deg", "%.2f", ShooterCalc.lastBurstHoodDeltaDeg);
        telemetry.addData("Solver iters / converged", "%d / %b", ShooterCalc.lastIterations, ShooterCalc.lastConverged);
        telemetry.addData("Hood", "%.2f", robot.hood.position());
        telemetry.addData("Latch", robot.latch.isOpen() ? "open" : "closed");
        telemetry.addData("Intake", robot.intake.getState());
        telemetry.addData("Loop ms", robot.loopMs());
        telemetry.update();
    }

    /** Panels graph tab: numbers only, so each one can be plotted over time. */
    private void panels(double turretOffset) {
        double targetRpm = robot.shooter.isSpinning() ? robot.shooter.targetRpm() : 0;
        panels.addData("Motor RPM", robot.shooter.rpm());
        panels.addData("Target RPM", targetRpm);
        panels.addData("RPM error", targetRpm - robot.shooter.rpm());
        panels.addData("Shooter power", robot.shooter.power());
        panels.addData("Battery V", robot.shooter.voltage());
        panels.addData("Shooting", robot.isShooting() ? 1 : 0);
        panels.addData("Hood pos", robot.hood.position());
        panels.addData("Nearest tuned point in", Double.isNaN(ShooterCalc.lastTunedNearestIn) ? -1 : ShooterCalc.lastTunedNearestIn);
        panels.addData("Range in", ShooterCalc.lastRange);
        panels.addData("Virtual range in", ShooterCalc.lastVirtualRange);
        panels.addData("Turret angle deg", ShooterCalc.lastHeadingAngle);
        panels.addData("Robot speed in/s (proj)", ShooterCalc.lastProjectedSpeed);
        panels.addData("Accel in/s^2 (filt)", ShooterCalc.lastAlongTrackAccel);
        panels.addData("Manual mode", robot.isManual() ? 1 : 0);
        panels.addData("Live RPM trim", Robot.LIVE_RPM_TRIM);
        panels.addData("Live hood trim", Robot.LIVE_HOOD_TRIM);
        panels.addData("Turret offset deg", turretOffset);
        panels.addData("Loop ms", robot.loopMs());
        panels.update();   // Panels only; the Driver Station lines above are sent by telemetry.update()
    }
}
