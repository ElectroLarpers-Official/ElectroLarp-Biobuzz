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
 *   <li>While feeding (setFeeding(true), done by Robot when it shoots): bang-bang, BANG_POWER whenever the wheel
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
    public static double kP = 0.0006;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 1.1;

    // Meta's feeding behaviour.
    public static boolean BANG_BANG_WHILE_FEEDING = false;
    public static double DEADBAND_RPM = 15;
    public static double BANG_POWER = 1.0;

    /** Only write motor power when it changes by more than this (Meta: 0.005). */
    public static double WRITE_THRESHOLD = 0.005;
    /** Battery voltage read interval, ms. Each read is its own hub transaction. */
    public static double VOLTAGE_PERIOD_MS = 100;

    /** "At speed" band for atTarget(), ticks/s. ShotCheck uses its own (shot-map) tolerance for READY. */
    public static double VELOCITY_TOLERANCE = 100; // TODO: tune

    public enum Mode { OFF, PIDF, BANG_BANG }

    private final DcMotorEx motor1, motor2;
    private final List<LynxModule> hubs;
    private final PIDController pid = new PIDController(kP, kI, kD, kF, true);
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double targetVelocity = 0;   // ticks/s
    private boolean feeding = false;
    private Mode mode = Mode.OFF;
    private double rpm = 0, power = 0, voltage = 12.0;
    private double lastWrittenPower = Double.NaN;

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
        else if (BANG_BANG_WHILE_FEEDING && feeding) next = Mode.BANG_BANG;
        else next = Mode.PIDF;
        // Fresh integral / derivative history whenever PIDF takes over.
        if (next == Mode.PIDF && mode != Mode.PIDF) pid.reset();
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
        writePower(power);
    }

    private void writePower(double p) {
        boolean changed = Double.isNaN(lastWrittenPower) || Math.abs(p - lastWrittenPower) > WRITE_THRESHOLD;
        boolean mustStop = p == 0 && lastWrittenPower != 0; // never leave a small leftover power on
        if (changed || mustStop) {
            motor1.setPower(p);
            motor2.setPower(p);
            lastWrittenPower = p;
        }
    }

    private void readVoltage() {
        voltageTimer.reset();
        if (!hubs.isEmpty()) voltage = hubs.get(0).getInputVoltage(VoltageUnit.VOLTS);
    }

    /** Target in motor encoder ticks/s. */
    public void setTargetVelocity(double ticksPerSecond) { targetVelocity = Math.max(0, ticksPerSecond); }
    public void setTargetRpm(double motorRpm) { setTargetVelocity(motorRpm * TICKS_PER_REV / 60.0); }
    /** True while balls are being fed: switches to bang-bang (if enabled). */
    public void setFeeding(boolean feeding) { this.feeding = feeding; }
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
    public boolean feeding() { return feeding; }
}
