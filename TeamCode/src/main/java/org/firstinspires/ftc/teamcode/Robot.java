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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Latch;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Transfer;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.control.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.control.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;

import com.pedropathing.math.Velocity;

import java.util.List;

/** Central owner for all robot subsystems, plus the commands that span more than one of them. */
@Configurable
public final class Robot {
    /** How long {@link #shootOne()} runs the transfer, i.e. long enough to pass exactly one ball. */
    public static double SHOOT_ONE_MS = 250;
    public final Alliance alliance;
    public final Follower follower;
    public final Intake intake;
    public final Transfer transfer;
    public final Latch latch;
    public final Turret turret;
    public final Shooter shooter;
    public final Hood hood;

    private final List<LynxModule> hubs;
    private long lastLoopNanos = 0;
    private double loopMs = 0;

    /** The alliance picks the goal the turret tracks, which in turn sets the flywheel speed and hood. */
    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        // Bulk reads: one hub transaction per loop instead of one per sensor/encoder read.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        latch = new Latch(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
    }

    /** Call once per loop, before anything reads the follower or the hub's bulk data. */
    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        follower.update(); // before aiming, so the solver sees this loop's pose and velocity
        updateAiming();
        shooter.update();

        long now = System.nanoTime();
        if (lastLoopNanos != 0) loopMs = (now - lastLoopNanos) / 1e6;
        lastLoopNanos = now;
    }

    /**
     *This is a meathod that is updated every loop to aim turret, set tps, and raise hood
     */
    private void updateAiming() {
        Pose currPose = follower.pose();
        Velocity vel = follower.velocity();
        Vector2D velocity = vel.toVector2D();
        Vector2D accel = Vector2D.zero();
        double angularVelocity = vel.omega;

        Pose turretPose = Turret.getTurretPose(currPose);
        Vector2D robotToGoal = Turret.getTurretToGoalVector(turretPose, alliance);

        Double[] result = ShooterCalc.solve(
                currPose.heading(), robotToGoal, velocity, accel, angularVelocity);

        shooter.setTargetVelocity(result[0]);
        hood.setPosition(result[1]);
        turret.setPosition(TurretCalc.solve(result[2], angularVelocity, alliance));
    }
    /**
     * Field-centric mecanum drive. Inputs are Pedro-signed: +forward, +lateral is left, +turn is counter-clockwise.
     * {@code forwardOffsetRad} rotates the frame so "forward" points away from the driver.
     */
    public void drive(double forward, double lateral, double turn, double forwardOffsetRad) {
        follower.manual(ManualDrive.fieldCentric(forward, lateral, turn, follower.pose().heading(), forwardOffsetRad));
    }

    /** Shoots everything held: opens the latch and runs the transfer and intake until {@link #stopShootAll()}. */
    public void shootAll() {
        latch.open();
        transfer.intake();
        intake.intake();
    }

    public void stopShootAll() {
        latch.close();
        transfer.stop();
        intake.stop();
    }

    /** Runs the intake and transfer in their SLOW states (slow reverse) to clear a jam. */
    public void outtake() {
        intake.slow();
        transfer.slow();
    }

    public void stopOuttake() {
        intake.stop();
        transfer.stop();
    }

    /** Passes exactly one ball: opens the latch, pulses the transfer, closes the latch. The intake is left off on purpose. */
    public CommandBuilder shootOne() {
        return sequential(
                instant(latch::open),
                instant(intake::slow),
                instant(transfer::slow),
                waitMs(SHOOT_ONE_MS),
                instant(transfer::stop),
                instant(latch::close)

        );
    }

    /** Puts every subsystem in a safe state. Call when the OpMode ends. */
    public void stop() {
        intake.stop();
        transfer.stop();
        latch.close();
        shooter.stop();
    }

    public double loopMs() {
        return loopMs;
    }
}
