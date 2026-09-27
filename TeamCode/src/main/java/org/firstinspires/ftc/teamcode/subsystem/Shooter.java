package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

/**
 * Single-motor flywheel. Velocity is held by feedforward + P in {@link #update()},
 * so that must be called every loop (Robot does this).
 */
@Configurable
public final class Shooter {
    /** Fixed flywheel target for party mode, ticks/s. Party mode is reserved for driver 2 controls, which are not added yet. */
    public static double PARTY_MODE_VELOCITY = 1800; // TODO: confirm real value

    /**
     * Gains carried over from the old NextFTC flywheel, which worked well.
     * TODO: retune on the real single-motor flywheel.
     */
    public static double kS = 0.2;        // static friction, applied whenever a target is set
    public static double kF = 0.000449;   // power per tick/s of target
    public static double kP = 0.009;      // power per tick/s of velocity error
    public static double VELOCITY_TOLERANCE = 100;

    private final CachedMotor motor;
    private double targetVelocity = 0;

    public Shooter(HardwareMap hw) {
        motor = new CachedMotor(hw, HardwareNames.SHOOTER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast down
    }

    public void update() {
        if (targetVelocity <= 0) {
            motor.setPower(0);
            return;
        }
        double power = kS + kF * targetVelocity + kP * (targetVelocity - motor.getVelocity());
        motor.setPower(Math.max(0, Math.min(1, power)));
    }

    /** Called every loop with the solver's requiredTPS during normal (non-party) operation. */
    public void setTargetVelocity(double ticksPerSecond) { targetVelocity = Math.max(0, ticksPerSecond); }
    /** Reserved for driver 2 (not wired up yet): holds {@link #PARTY_MODE_VELOCITY} instead of the solver's target. */
    public void partyMode() { setTargetVelocity(PARTY_MODE_VELOCITY); }
    public void stop() {
        targetVelocity = 0;
        motor.setPower(0);
    }

    public boolean isSpinning() { return targetVelocity > 0; }
    public boolean atTarget() { return isSpinning() && Math.abs(motor.getVelocity() - targetVelocity) <= VELOCITY_TOLERANCE; }
    public double velocity() { return motor.getVelocity(); }
    public double targetVelocity() { return targetVelocity; }
}