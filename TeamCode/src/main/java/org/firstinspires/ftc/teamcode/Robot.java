package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.PulseRuntime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Flower;
import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Latch;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import com.pedropathing.math.Velocity;
import java.util.List;


@Configurable
public final class Robot {
    /** How long shootCycle() keeps the latch open and the transfer running, ms. */
    public static double SHOOT_DELAY_MS = 250; // TODO: tune
    /** Drive all 4 turret servos (turretServo1-4 on their own ports, directions in Turret). Off = only
     *  turretServo1, e.g. with the Y-split or to run on one servo. Read when the OpMode is initialised. */
    public static boolean TURRET_ALL_SERVOS = true; // TODO delete (quad turret)

    // ---- Driver 2 live trims: added to every shot (tuned points and manual mode) -------------------------------
    /** Motor RPM added to the flywheel target. Driver 2 d-pad up / down in TeleOp. */
    public static double LIVE_RPM_TRIM = 0;
    /** Hood servo position added to the hood target (+ = steeper). Driver 2 d-pad right / left in TeleOp. */
    public static double LIVE_HOOD_TRIM = 0;

    // ---- Manual fallback (driver 2 X): for when the pose can't be trusted ----------------------------------
    // The turret points where driver 2 puts it (straight ahead to start), the flywheel runs MANUAL_RPM
    // (+ LIVE_RPM_TRIM) and the hood sits at MANUAL_HOOD (+ LIVE_HOOD_TRIM).
    public static double MANUAL_RPM = 4325, MANUAL_HOOD = 0.435;
    /** Turret speed under driver 2's right stick in manual mode, deg/s at full stick. */
    public static double MANUAL_TURRET_RATE_DEG_S = 120;

    // ---- Pose reset (driver 1 X) ---------------------------------------------------------------------------------
    /** Where the robot is when driver 1 presses X, Pedro inches / degrees: the red alliance corner with the robot
     *  squared against both walls. SET THESE for the robot: x / y = distance from each wall to the pose point,
     *  heading = which way the robot faces there. The pose point is the centre of rotation, not the middle of the
     *  frame. From CAD it is 6.92 in from the backplate, 9.57 in from the front of the side panels, 6.86 in from
     *  each side panel and 7.56 in from the outside of the flower pivots. Blue uses
     *  the same spot turned 180 degrees about the field centre (144 - x, 144 - y, heading + 180). */
    public static double RESET_X = 9, RESET_Y = 9, RESET_HEADING_DEG = 90;

    public final Alliance alliance;
    public final PulseFollower follower;
    public final Intake intake;

    public Flower flower;
    public final Latch latch;
    public final Turret turret;
    public final Shooter shooter;
    public final Hood hood;

    private final List<LynxModule> hubs;
    private long shootStartNanos = 0;
    private boolean underHive = false;
    private long lastLoopNanos = 0;
    private double loopMs = 0;
    private boolean manual = false;
    private double manualTurretDeg = 0;


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        PulseRuntime.attach((PulseFollower) Constants.create(hardwareMap));
        this.alliance = alliance;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        flower = new Flower(hardwareMap);

        follower = PulseRuntime.follower();
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        turret = new Turret(hardwareMap, TURRET_ALL_SERVOS); // TODO delete: -> new Turret(hardwareMap)
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        ShooterCalc.resetProjectionFilter();
        TurretCalc.reset();
    }


    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update();
        updateAiming();
        shooter.update();

        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;
    }

    /** Driver 1 A / Y: open / close the latch by hand. */
    public void manualLatch(boolean open) {
        if (open) latch.open(); else latch.close();
    }

    private void updateAiming() {
        Pose currPose = follower.pose();
        Velocity vel = follower.velocity();
        Vector2D velocity = vel.toVector2D();
        double angularVelocity = vel.omega;

        // Manual fallback: the pose isn't trusted, so no solver and no under-HIVE box.
        if (manual) {
            underHive = false;
            shooter.setTargetRpm(MANUAL_RPM + LIVE_RPM_TRIM);
            hood.setPosition(MANUAL_HOOD + LIVE_HOOD_TRIM);
            turret.setPositionAll(TurretCalc.solve(turretForward ? 0.0 : manualTurretDeg, 0.0, alliance)); // TODO delete: -> setPosition
            return;
        }

        Pose turretPose = Turret.getTurretPose(currPose);
        Pose goal = ShooterCalc.goal(alliance, currPose.y());
        Vector2D robotToGoal = Turret.getTurretToGoalVector(turretPose, goal);

        // Under the HIVE: don't run the solver (no shot exists there). Hold a fixed flywheel speed and hood, keep the
        // turret pointed at the cell so it's ready on the way out.
        if (ShooterConstants.underHive(currPose.x(), currPose.y())) {
            underHive = true;
            shooter.setTargetRpm(ShooterConstants.UNDER_HIVE_RPM);
            hood.setLaunchAngle(ShooterConstants.UNDER_HIVE_LAUNCH_DEG);
            double bearingDeg = Math.toDegrees(robotToGoal.theta() - currPose.heading());
            // TODO delete: -> setPosition
            turret.setPositionAll(turretForward ? TurretCalc.solve(0.0, 0.0, alliance)
                    : TurretCalc.solve(ShooterCalc.normalizeDegrees(bearingDeg), angularVelocity, alliance));
            return;
        }
        underHive = false;

        double shootMs = (shootStartNanos == 0) ? 0.0 : (System.nanoTime() - shootStartNanos) / 1e6;
        ShooterCalc.burstCompActive = shootStartNanos != 0 && shootMs >= ShooterCalc.burstCompDelayMs;
        // Measured flywheel speed for the burst comp, live trim taken off so it compares with the solver's target.
        ShooterCalc.flywheelTps = shooter.velocity() - LIVE_RPM_TRIM * Shooter.TICKS_PER_REV / 60.0;

        double[] result = ShooterCalc.solve(currPose.heading(), robotToGoal, velocity, angularVelocity);

        double tps = result[0];
        if (tps > 0) tps = Math.max(0, tps + LIVE_RPM_TRIM * Shooter.TICKS_PER_REV / 60.0);
        shooter.setTargetVelocity(tps);
        if (!Double.isNaN(result[1])) hood.setPosition(result[1] + LIVE_HOOD_TRIM);   // NaN: no tuned points
        turret.setPositionAll(turretForward ? TurretCalc.solve(0.0, 0.0, alliance) // driver 2: straight ahead
                : TurretCalc.solve(result[2], angularVelocity, alliance)); // TODO delete: -> setPosition
    }

    // ---- Manual fallback ---------------------------------------------------------------------------------------
    public boolean isManual() { return manual; }
    private boolean turretForward = false;
    /** Driver 2's turret-forward toggle: turret held straight ahead (0 deg + the alliance turret trim). */
    public boolean isTurretForward() { return turretForward; }
    public void setTurretForward(boolean on) { turretForward = on; }
    /** Turn manual mode on / off. Turning it on starts with the turret straight ahead. */
    public void setManual(boolean on) {
        if (on && !manual) manualTurretDeg = 0;
        manual = on;
    }
    public double manualTurretDeg() { return manualTurretDeg; }
    /** Turns the manual turret aim by {@code deg} (+ = counter-clockwise), inside the turret's travel. */
    public void nudgeManualTurret(double deg) {
        manualTurretDeg = Math.max(TurretCalc.MIN_ANGLE, Math.min(TurretCalc.MAX_ANGLE, manualTurretDeg + deg));
    }

    // ---- Pose reset ------------------------------------------------------------------------------------------
    /** The pose reset spot for an alliance (RESET_X / Y / HEADING_DEG for red, turned 180 deg for blue). */
    public static Pose resetPose(Alliance alliance) {
        if (alliance == Alliance.RED) return new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_HEADING_DEG));
        return new Pose(144 - RESET_X, 144 - RESET_Y, Math.toRadians(RESET_HEADING_DEG + 180));
    }
    /** Tells the localizer the robot is at the reset spot now. */
    public void resetPose() {
        follower.setPose(resetPose(alliance));
    }

    /** True while the robot is under the HIVE (solver skipped, fixed RPM / hood). */
    public boolean underHive() {
        return underHive;
    }


    public void drive(double forward, double lateral, double turn, double forwardOffsetRad) {
        follower.manual(ManualDrive.fieldCentric(forward, lateral, turn, follower.pose().heading(), forwardOffsetRad));
    }

    /** Open latch, transfer on. Stays on until stopShooting() (Tele right trigger hold). */
    public void startShooting() {
        shootStartNanos = System.nanoTime();
        latch.open();
        intake.on();
        shooter.setShooting(true);
    }

    /** Transfer off, close latch. */
    public void stopShooting() {
        shootStartNanos = 0;
        intake.off();
        latch.close();
        shooter.setShooting(false);
    }

    /** True between startShooting() and stopShooting(). */
    public boolean isShooting() {
        return shootStartNanos != 0;
    }

    /** Open latch, transfer on, wait SHOOT_DELAY_MS, transfer off, close latch. */
    public Command shootCycle() {
        return sequential(
                instant(this::startShooting),
                waitMs(SHOOT_DELAY_MS),
                instant(this::stopShooting)
        );
    }

    public void flower(boolean down) {
        flower.setDown(down);
    }

    public void stop() {
        shootStartNanos = 0;
        intake.off();
        latch.close();
        shooter.setShooting(false);
        shooter.stop();
    }

    public double loopMs() {
        return loopMs;
    }
}
