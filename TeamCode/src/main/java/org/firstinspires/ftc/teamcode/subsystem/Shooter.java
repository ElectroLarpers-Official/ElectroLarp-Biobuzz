package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.control.PIDController;

import java.util.List;

/**
 * Flywheel: two motors (20T GT2 -> 40T flywheel), encoder on the first. Meta Infinity's control, as tuned in
 * FlywheelPIDFTest:
 * <ul>
 *   <li>Normally PIDF on motor RPM (Meta's PIDController in velocity mode): feedforward = kF * targetRPM / 5200,
 *       plus kP / kI / kD on the RPM error, all scaled by 12 V / battery voltage.</li>
 *   <li>While shooting (setShooting(true), done by Robot when it shoots): bang-bang, BANG_POWER whenever the wheel
 *       is more than DEADBAND_RPM under target, else 0. This recovers the wheel between balls.</li>
 * </ul>
 * The rest of the code still talks in motor encoder ticks/s (setTargetVelocity, velocity()); this class converts.
 */
@Configurable
public final class Shooter {
    /** Fixed flywheel target for party mode, ticks/s. Party mode is reserved for driver 2 controls, which are not added yet. */
    public static double PARTY_MODE_VELOCITY = 1800; // TODO: confirm real value

    public static boolean MOTORS_REVERSED = true;
    /** Flip if the RPM reads negative while the wheel spins the right way. */
    public static boolean ENCODER_REVERSED = false;
    /** Encoder counts per motor revolution (goBILDA 5203). */
    public static double TICKS_PER_REV = 28;

    // PIDF, tuned in FlywheelPIDFTest. kF = 1 is full power at 5200 motor RPM (hardcoded in PIDController).
    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 1;

    // Meta's behaviour while balls go through the wheel.
    public static boolean BANG_BANG_WHILE_SHOOTING = false;
    public static double DEADBAND_RPM = 15;
    public static double BANG_POWER = 1.0;

    /** Only write motor power when it changes by more than this (Meta: 0.005). While shooting every change is
     *  written, so the wheel recovers between balls as fast as possible. */
    public static double WRITE_THRESHOLD = 0.005;
    /** A change under WRITE_THRESHOLD is still written once the last write is this old, ms. Keeps the loop-time
     *  saving while stopping the threshold from leaving a permanent deadband in the PIDF loop. */
    public static double WRITE_MAX_AGE_MS = 20;
    /** Battery voltage read interval, ms. Each read is its own hub transaction (about 1-3 ms), and the voltage only
     *  feeds the feedforward compensation, which doesn't need to follow every sag. */
    public static double VOLTAGE_PERIOD_MS = 1000;

    /** "At speed" band for atTarget(), ticks/s. ShotCheck uses its own (shot-map) tolerance for READY. */
    public static double VELOCITY_TOLERANCE = 100; // TODO: tune

    /** Spin-up ramp: when the flywheel starts from a stop, its motor power may rise by at most this much per
     *  second (0 -> full power in 1 / this, seconds), so the start doesn't sag the battery. Only the initial
     *  spin-up: once the wheel is near target (or the ramp reaches full power) the PIDF runs unlimited, so recovery
     *  between balls is unaffected. 0 = off. */
    public static double SPINUP_RAMP_PER_SEC = 1.5;
    /** The ramp only starts if the wheel is slower than this (motor RPM) when it's switched on. */
    public static double SPINUP_START_RPM = 500;
    /** The ramp ends once the wheel reaches this fraction of the target speed. */
    public static double SPINUP_DONE_FRACTION = 0.9;

    public enum Mode { OFF, PIDF, BANG_BANG }

    private final DcMotorEx motor1, motor2;
    private final List<LynxModule> hubs;
    private final PIDController pid = new PIDController(kP, kI, kD, kF, true);
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime writeTimer = new ElapsedTime();

    private double targetVelocity = 0;   // ticks/s
    private boolean shooting = false;
    private Mode mode = Mode.OFF;
    private double rpm = 0, power = 0, voltage = 12.0;
    private double lastWrittenPower = Double.NaN;
    private boolean spinningUp = false;
    private double rampCap = 0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    public Shooter(HardwareMap hw) {
        // Plain DcMotorEx, not CachedMotor: CachedMotor skips power changes under 0.01, Meta used 0.005.
        motor1 = hw.get(DcMotorEx.class, HardwareNames.SHOOTER);
        motor2 = hw.get(DcMotorEx.class, HardwareNames.SHOOTER_2);
        for (DcMotorEx m : new DcMotorEx[]{motor1, motor2}) {
            m.setDirection(MOTORS_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // still reads the encoder; the loop is closed here
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        hubs = hw.getAll(LynxModule.class);
        readVoltage();
    }

    /** Call once per loop, after the bulk cache is cleared. */
    public void update() {
        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) readVoltage();
        rpm = velocity() * 60.0 / TICKS_PER_REV;
        double targetRpm = targetRpm();

        Mode next;
        if (targetVelocity <= 0) next = Mode.OFF;
        else if (BANG_BANG_WHILE_SHOOTING && shooting) next = Mode.BANG_BANG;
        else next = Mode.PIDF;
        // Fresh integral / derivative history whenever PIDF takes over.
        if (next == Mode.PIDF && mode != Mode.PIDF) pid.reset();
        // Starting from a stop: begin the spin-up ramp.
        if (mode == Mode.OFF && next != Mode.OFF && rpm < SPINUP_START_RPM && SPINUP_RAMP_PER_SEC > 0) {
            spinningUp = true;
            rampCap = 0;
            rampTimer.reset();
        }
        if (next == Mode.OFF) spinningUp = false;
        mode = next;

        switch (mode) {
            case BANG_BANG:
                power = rpm < targetRpm - DEADBAND_RPM ? BANG_POWER : 0;
                break;
            case PIDF:
                pid.setGains(kP, kI, kD, kF);
                power = pid.update(targetRpm, rpm, voltage);
                break;
            default:
                power = 0;
        }
        if (spinningUp) {
            double dt = Math.min(rampTimer.seconds(), 0.1);
            rampTimer.reset();
            rampCap += SPINUP_RAMP_PER_SEC * dt;
            if (power > rampCap) power = rampCap;
            if (rampCap >= 1.0 || rpm >= SPINUP_DONE_FRACTION * targetRpm || SPINUP_RAMP_PER_SEC <= 0) spinningUp = false;
        }
        writePower(power);
    }

    private void writePower(double p) {
        if (p == lastWrittenPower) return; // nothing to send
        double threshold = shooting ? 0.0 : WRITE_THRESHOLD;
        boolean bigChange = Double.isNaN(lastWrittenPower) || Math.abs(p - lastWrittenPower) > threshold;
        boolean endpoint = p == 0 || Math.abs(p) >= 1;                // stop and full power are always exact
        boolean stale = writeTimer.milliseconds() >= WRITE_MAX_AGE_MS; // small drift can't wait forever
        if (bigChange || endpoint || stale) {
            motor1.setPower(p);
            motor2.setPower(p);
            lastWrittenPower = p;
            writeTimer.reset();
        }
    }

    private void readVoltage() {
        voltageTimer.reset();
        if (!hubs.isEmpty()) voltage = hubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
    }

    /** Target in motor encoder ticks/s. */
    public void setTargetVelocity(double ticksPerSecond) { targetVelocity = Math.max(0, ticksPerSecond); }
    public void setTargetRpm(double motorRpm) { setTargetVelocity(motorRpm * TICKS_PER_REV / 60.0); }
    /** True while balls are going through the wheel: switches to bang-bang (if enabled). */
    public void setShooting(boolean shooting) { this.shooting = shooting; }
    public void partyMode() { setTargetVelocity(PARTY_MODE_VELOCITY); }
    public void stop() {
        targetVelocity = 0;
        mode = Mode.OFF;
        power = 0;
        writePower(0);
    }

    public boolean isSpinning() { return targetVelocity > 0; }
    public boolean atTarget() { return isSpinning() && Math.abs(velocity() - targetVelocity) <= VELOCITY_TOLERANCE; }
    /** Measured motor speed, ticks/s. */
    public double velocity() { return (ENCODER_REVERSED ? -1 : 1) * motor1.getVelocity(); }
    public double targetVelocity() { return targetVelocity; }
    public double rpm() { return rpm; }
    public double targetRpm() { return targetVelocity * 60.0 / TICKS_PER_REV; }
    public double power() { return power; }
    public double voltage() { return voltage; }
    public Mode mode() { return mode; }
    public boolean isShooting() { return shooting; }
}
