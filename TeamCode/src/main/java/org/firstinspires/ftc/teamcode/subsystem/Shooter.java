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


@Configurable
public final class Shooter {
    public static double PARTY_MODE_VELOCITY = 1800;
    public static boolean MOTORS_REVERSED = true;
    public static boolean ENCODER_REVERSED = false;
    public static double TICKS_PER_REV = 28;

    public static double kP = 0.001;
    public static double kP_FEEDING = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.98;

    public static double WRITE_THRESHOLD = 0.005;
    public static double WRITE_MAX_AGE_MS = 20;
    public static double VOLTAGE_PERIOD_MS = 1000;


    public static double VELOCITY_TOLERANCE = 100; // TODO: tune
    public static double SPINUP_RAMP_PER_SEC = 1.5;

    public static double SPINUP_START_RPM = 500;

    public static double SPINUP_DONE_FRACTION = 0.9;

    public enum Mode { OFF, PIDF }

    private final DcMotorEx motor1, motor2;
    private final List<LynxModule> hubs;
    private final PIDController pid = new PIDController(kP, kI, kD, kF, true);
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime writeTimer = new ElapsedTime();

    private double targetVelocity = 0;
    private boolean shooting = false;
    private Mode mode = Mode.OFF;
    private double rpm = 0, power = 0, voltage = 12.0;
    private double lastWrittenPower = Double.NaN;
    private boolean spinningUp = false;
    private double rampCap = 0;
    private final ElapsedTime rampTimer = new ElapsedTime();

    public Shooter(HardwareMap hw) {
        motor1 = hw.get(DcMotorEx.class, HardwareNames.SHOOTER);
        motor2 = hw.get(DcMotorEx.class, HardwareNames.SHOOTER_2);
        for (DcMotorEx m : new DcMotorEx[]{motor1, motor2}) {
            m.setDirection(MOTORS_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        hubs = hw.getAll(LynxModule.class);
        readVoltage();
    }

    public void update() {
        if (voltageTimer.milliseconds() >= VOLTAGE_PERIOD_MS) readVoltage();
        rpm = velocity() * 60.0 / TICKS_PER_REV;
        double targetRpm = targetRpm();

        Mode next;
        if (targetVelocity <= 0) next = Mode.OFF;
        else next = Mode.PIDF;
        if (next == Mode.PIDF && mode != Mode.PIDF) pid.reset();
        if (mode == Mode.OFF && next != Mode.OFF && rpm < SPINUP_START_RPM && SPINUP_RAMP_PER_SEC > 0) {
            spinningUp = true;
            rampCap = 0;
            rampTimer.reset();
        }
        if (next == Mode.OFF) spinningUp = false;
        mode = next;

        switch (mode) {
            case PIDF:
                pid.setGains(shooting ? kP_FEEDING : kP, kI, kD, kF);
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
        boolean endpoint = p == 0 || Math.abs(p) >= 1;
        boolean stale = writeTimer.milliseconds() >= WRITE_MAX_AGE_MS;
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


    public void setTargetVelocity(double ticksPerSecond) { targetVelocity = Math.max(0, ticksPerSecond); }
    public void setTargetRpm(double motorRpm) { setTargetVelocity(motorRpm * TICKS_PER_REV / 60.0); }

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
    public double velocity() { return (ENCODER_REVERSED ? -1 : 1) * motor1.getVelocity(); }
    public double targetVelocity() { return targetVelocity; }
    public double rpm() { return rpm; }
    public double targetRpm() { return targetVelocity * 60.0 / TICKS_PER_REV; }
    public double power() { return power; }
    public double voltage() { return voltage; }
    public Mode mode() { return mode; }
    public boolean isShooting() { return shooting; }
}
