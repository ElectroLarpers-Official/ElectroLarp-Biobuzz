package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.electrolightsftc.pulse.electropathing.PulseFollower;
import org.electrolightsftc.pulse.electropathing.PulseRuntime;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Latch;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
// TODO: pick one shooter logic version (control.shotcheck or control.pasted) and delete the other; until then these imports must match Tele's.
import org.firstinspires.ftc.teamcode.subsystem.control.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShotCheck;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import com.pedropathing.math.Velocity;
import java.util.List;


@Configurable
public final class Robot {
    public static double SHOOT_ONE_MS = 250; // TODO: tune
    public final Alliance alliance;
    public final PulseFollower follower;
    public final Intake intake;
    public final Latch latch;
    public final Turret turret;
    public final Shooter shooter;
    public final Hood hood;

    private final List<LynxModule> hubs;
    private long feedStartNanos = 0;
    private boolean underHive = false;
    private long lastLoopNanos = 0;
    private double loopMs = 0;


    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        PulseRuntime.attach((PulseFollower) Constants.create(hardwareMap));
        this.alliance = alliance;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = PulseRuntime.follower();
        intake = new Intake(hardwareMap);
        latch = new Latch(hardwareMap);
        turret = new Turret(hardwareMap);
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

    private void updateAiming() {
        Pose currPose = follower.pose();
        Velocity vel = follower.velocity();
        Vector2D velocity = vel.toVector2D();
        Vector2D accel = Vector2D.zero();
        double angularVelocity = vel.omega;

        Pose turretPose = Turret.getTurretPose(currPose);
        Pose goal = ShooterCalc.goal(alliance, currPose.y());
        Vector2D robotToGoal = Turret.getTurretToGoalVector(turretPose, goal);

        // Under the HIVE: don't run the solver (no shot exists there). Hold a fixed flywheel speed and hood, keep the
        // turret pointed at the cell so it's ready on the way out, and block shooting.
        if (ShooterCalc.hiveMode && ShooterConstants.underHive(currPose.x(), currPose.y())) {
            underHive = true;
            shooter.setTargetRpm(ShooterConstants.UNDER_HIVE_RPM);
            hood.setLaunchAngle(ShooterConstants.UNDER_HIVE_LAUNCH_DEG);
            double bearingDeg = Math.toDegrees(robotToGoal.theta() - currPose.heading());
            turret.setPosition(TurretCalc.solve(ShooterCalc.normalizeDegrees(bearingDeg), angularVelocity, alliance));
            ShotCheck.blockUnderHive();
            return;
        }
        underHive = false;

        double feedMs = (feedStartNanos == 0) ? 0.0 : (System.nanoTime() - feedStartNanos) / 1e6;
        ShooterCalc.burstCompActive = feedStartNanos != 0 && feedMs >= ShooterCalc.burstCompDelayMs;

        Double[] result = ShooterCalc.solve(
                currPose.heading(), robotToGoal, velocity, accel, angularVelocity);

        shooter.setTargetVelocity(result[0]);
        hood.setPosition(result[1]);
        turret.setPosition(TurretCalc.solve(result[2], angularVelocity, alliance));
        ShotCheck.check(alliance, goal);
    }

    /** True while the robot is under the HIVE (solver skipped, fixed RPM / hood). */
    public boolean underHive() {
        return underHive;
    }

    public boolean shotAllowed() {
        return ShotCheck.allowed();
    }


    public boolean readyToShoot() {
        return ShotCheck.ready(shooter.velocity(), shooter.targetVelocity());
    }

    public void drive(double forward, double lateral, double turn, double forwardOffsetRad) {
        follower.manual(ManualDrive.fieldCentric(forward, lateral, turn, follower.pose().heading(), forwardOffsetRad));
    }

    public void shootAll() {
        feedStartNanos = System.nanoTime();
        latch.open();
        shooter.setFeeding(true);
        intake.intake();
    }

    /** True while shootAll is feeding (right trigger). */
    public boolean isShooting() {
        return feedStartNanos != 0;
    }

    public void stopShootAll() {
        feedStartNanos = 0;
        latch.close();
        shooter.setFeeding(false);
        intake.stop();
    }

    public void outtake() {
        intake.slow();
    }

    public void stopOuttake() {
        intake.stop();
    }

    public CommandBuilder shootOne() {
        return sequential(
                instant(latch::open),
                instant(() -> shooter.setFeeding(true)),
                instant(intake::intake),
                waitMs(SHOOT_ONE_MS),
                instant(intake::stop),
                instant(() -> shooter.setFeeding(false)),
                instant(latch::close)
        );
    }

    public void stop() {
        feedStartNanos = 0;
        intake.stop();
        latch.close();
        shooter.setFeeding(false);
        shooter.stop();
    }

    public double loopMs() {
        return loopMs;
    }
}
