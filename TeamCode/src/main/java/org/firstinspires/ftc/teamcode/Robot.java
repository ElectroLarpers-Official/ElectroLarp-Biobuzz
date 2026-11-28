package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.PulseRuntime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Flower;
import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Latch;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.StickLead;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.FollowerLocalization;
import org.firstinspires.ftc.teamcode.util.ShotLogger;
import com.pedropathing.math.Velocity;
import java.util.List;


@Configurable
public final class Robot {
    public static double SHOOT_DELAY_MS = 250; // TODO: tune
    public static boolean TURRET_ALL_SERVOS = true; // TODO delete (quad turret)
    public static double LIVE_RPM_TRIM = 25;
    public static double LIVE_HOOD_TRIM = 0;
    public static double MANUAL_RPM = 6000, MANUAL_HOOD = 0.435;
    public static double MANUAL_TURRET_RATE_DEG_S = 120;
    public static double RESET_X = 9, RESET_Y = 9, RESET_HEADING_DEG = 90;

    public final Alliance alliance;
    public final PulseFollower follower;
    public final Intake intake;

    public Flower flower;
    public final Latch latch;
    public final Turret turret;
    public final Shooter shooter;
    public final Hood hood;
    public final Drivetrain drivetrain;
    private boolean teleOpDrive = false;

    private final List<LynxModule> hubs;
    private long shootStartNanos = 0;
    private boolean underHive = false;
    private long lastLoopNanos = 0;
    private double loopMs = 0;
    private boolean manual = false;
    private double manualTurretDeg = 0;
    private Double turretOffsetOverrideDeg;
    private Double fixedTurretAngleDeg;
    private final StickLead stickLead = new StickLead();
    private long lastAimNanos = 0;
    private final ShotLogger shotLog = new ShotLogger();


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        PulseRuntime.attach((PulseFollower) Constants.create(hardwareMap));
        this.alliance = alliance;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        flower = new Flower(hardwareMap);

        follower = PulseRuntime.follower();
        drivetrain = new Drivetrain(hardwareMap, follower, alliance);
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        turret = new Turret(hardwareMap, TURRET_ALL_SERVOS);
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        ShooterCalc.resetProjectionFilter();
        TurretCalc.reset();
    }


    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        if (teleOpDrive) FollowerLocalization.update(follower);
        else follower.update();
        updateAiming();
        shooter.update();
        flower.update();
        stickLead.clearSticks();
        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;
        if (ShotLogger.SHOT_LOG) logShot(now);
    }

    public void setDriverSticks(double forward, double lateral, double turn, double forwardOffsetRad) {
        stickLead.setSticks(forward, lateral, turn, forwardOffsetRad, !drivetrain.isFieldCentric());
    }

    public double stickBlendDeg() { return stickLead.changeDeg(); }

    private void logShot(long nowNanos) {
        Pose pose = follower.pose();
        Velocity vel = follower.velocity();
        Vector2D v = vel.toVector2D();
        double speed = v.magnitude();
        double[] row = shotLog.row();
        row[ShotLogger.T] = nowNanos * 1e-9;
        row[ShotLogger.LOOP_MS] = loopMs;
        row[ShotLogger.X] = pose.x();
        row[ShotLogger.Y] = pose.y();
        row[ShotLogger.HEADING_DEG] = Math.toDegrees(pose.heading());
        row[ShotLogger.VX] = speed * Math.cos(v.theta());
        row[ShotLogger.VY] = speed * Math.sin(v.theta());
        row[ShotLogger.SPEED] = speed;
        row[ShotLogger.OMEGA] = vel.omega;
        row[ShotLogger.RANGE] = ShooterCalc.lastRange;
        row[ShotLogger.VIRTUAL_RANGE] = ShooterCalc.lastVirtualRange;
        row[ShotLogger.FLIGHT_TIME] = ShooterCalc.lastFlightTime;
        row[ShotLogger.TURRET_CMD_DEG] = TurretCalc.commandDeg();
        row[ShotLogger.RPM_TARGET] = shooter.targetRpm();
        row[ShotLogger.RPM_ACTUAL] = shooter.rpm();
        row[ShotLogger.META_MODE] = ShooterCalc.USE_META_SOTM ? 1 : 0;
        row[ShotLogger.SHOOTING] = isShooting() ? 1 : 0;
        row[ShotLogger.STICK_BLEND_DEG] = stickLead.changeDeg();
        shotLog.commit(isShooting());
    }

    public void manualLatch(boolean open) {
        if (open) latch.open(); else latch.close();
    }

    public void setTurretOffsetOverride(double offsetDeg) {
        turretOffsetOverrideDeg = offsetDeg;
    }

    public void setFixedTurretAngle(double angleDeg) {
        fixedTurretAngleDeg = angleDeg;
    }

    public Double fixedTurretAngleDeg() { return fixedTurretAngleDeg; }

    private double solveTurret(double angleDeg, double angularVelocity) {
        if (fixedTurretAngleDeg != null) {
            return TurretCalc.solveWithoutWrap(fixedTurretAngleDeg, 0.0, 0.0);
        }
        return turretOffsetOverrideDeg == null
                ? TurretCalc.solve(angleDeg, angularVelocity, alliance)
                : TurretCalc.solve(angleDeg, angularVelocity, turretOffsetOverrideDeg);
    }

    private void updateAiming() {
        Pose currPose = follower.pose();
        Velocity vel = follower.velocity();
        Vector2D velocity = vel.toVector2D();
        double angularVelocity = vel.omega;


        long aimNanos = System.nanoTime();
        double aimDt = lastAimNanos == 0 ? 0.0 : Math.min(0.1, (aimNanos - lastAimNanos) / 1e9);
        lastAimNanos = aimNanos;
        stickLead.track(aimDt);
        stickLead.apply(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   // not acting unless the normal shot below runs


        if (manual) {
            underHive = false;
            shooter.setTargetRpm(MANUAL_RPM + LIVE_RPM_TRIM);
            hood.setPosition(MANUAL_HOOD + LIVE_HOOD_TRIM);
            turret.setPositionAll(solveTurret(turretForward ? 0.0 : manualTurretDeg, 0.0));
            return;
        }

        Pose turretPose = Turret.getTurretPose(currPose);
        Pose goal = ShooterCalc.goal(alliance, currPose.y());
        Vector2D robotToGoal = Turret.getTurretToGoalVector(turretPose, goal);

        if (ShooterConstants.underHive(currPose.x(), currPose.y())) {
            underHive = true;
            shooter.setTargetRpm(ShooterConstants.UNDER_HIVE_RPM);
            hood.setLaunchAngle(ShooterConstants.UNDER_HIVE_LAUNCH_DEG);
            double bearingDeg = Math.toDegrees(robotToGoal.theta() - currPose.heading());
            // TODO delete: -> setPosition
            turret.setPositionAll(turretForward ? solveTurret(0.0, 0.0)
                    : solveTurret(ShooterCalc.normalizeDegrees(bearingDeg), angularVelocity));
            return;
        }
        underHive = false;

        double shootMs = (shootStartNanos == 0) ? 0.0 : (System.nanoTime() - shootStartNanos) / 1e6;
        ShooterCalc.burstCompActive = shootStartNanos != 0 && shootMs >= ShooterCalc.burstCompDelayMs;

        ShooterCalc.flywheelTps = shooter.velocity() - LIVE_RPM_TRIM * Shooter.TICKS_PER_REV / 60.0;

        double[] result = ShooterCalc.solve(currPose.heading(), robotToGoal, velocity, angularVelocity);

        double tps = result[0];
        if (tps > 0) tps = Math.max(0, tps + LIVE_RPM_TRIM * Shooter.TICKS_PER_REV / 60.0);
        shooter.setTargetVelocity(tps);
        if (!Double.isNaN(result[1])) hood.setPosition(result[1] + LIVE_HOOD_TRIM);


        double speed = velocity.magnitude();
        stickLead.apply(result[2], ShooterCalc.lastVirtualRange, currPose.heading(),
                speed * Math.cos(velocity.theta()), speed * Math.sin(velocity.theta()), angularVelocity,
                TurretCalc.leadSec());
        turret.setPositionAll(turretForward ? solveTurret(0.0, 0.0)
                : solveTurret(stickLead.aimDeg(), stickLead.omega())); // TODO delete: -> setPosition
    }

    // ---- Manual fallback ---------------------------------------------------------------------------------------
    public boolean isManual() { return manual; }
    private boolean turretForward = false;
    public boolean isTurretForward() { return turretForward; }
    public void setTurretForward(boolean on) { turretForward = on; }

    public void setManual(boolean on) {
        if (on && !manual) manualTurretDeg = 0;
        manual = on;
    }
    public double manualTurretDeg() { return manualTurretDeg; }
    public void nudgeManualTurret(double deg) {
        manualTurretDeg = Math.max(TurretCalc.MIN_ANGLE, Math.min(TurretCalc.MAX_ANGLE, manualTurretDeg + deg));
    }

    // ---- Pose reset ------------------------------------------------------------------------------------------
    public static Pose resetPose(Alliance alliance) {
        if (alliance == Alliance.RED) return new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_HEADING_DEG));
        return new Pose(144 - RESET_X, 144 - RESET_Y, Math.toRadians(RESET_HEADING_DEG + 180));
    }
    public void resetPose() {
        follower.setPose(resetPose(alliance));
    }

    public boolean underHive() {
        return underHive;
    }


    public void startTeleOpDrive() {
        teleOpDrive = true;
        drivetrain.startTeleOpDrive();
    }

    public void startShooting() {
        shootStartNanos = System.nanoTime();
        latch.open();
        intake.on();
        shooter.setShooting(true);
    }

    public void stopShooting() {
        shootStartNanos = 0;
        intake.off();
        latch.close();
        shooter.setShooting(false);
    }


    public boolean isShooting() {
        return shootStartNanos != 0;
    }


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
        shotLog.save();
        //clearPause();
        shootStartNanos = 0;
        intake.off();
        latch.close();
        shooter.setShooting(false);
        shooter.stop();
        flower.stop();
    }

    public double loopMs() {
        return loopMs;
    }

}
