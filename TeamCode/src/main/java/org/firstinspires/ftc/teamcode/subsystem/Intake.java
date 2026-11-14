package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

/**
 * Intake + transfer. They are belted together on the robot, so they run as one subsystem: the "intake" motor and,
 * if it is in the config, the "transfer" motor, always at the same power.
 * Directions as tuned in FlywheelPIDFTest: intake motor reversed, transfer motor forward.
 */
@Configurable
public final class Intake {
    public static boolean REVERSED = true;            // FlywheelPIDFTest ran the intake at -1
    public static boolean TRANSFER_REVERSED = false;  // and the transfer at +1

    public static double ON_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;
    public static double SLOW_REVERSE_POWER = -0.5; // used to clear a jam

    public enum IntakeState {
        ON,
        REVERSE,
        SLOW_REVERSE,
        OFF
    }

    private final CachedMotor motor;
    private final CachedMotor transferMotor; // null if "transfer" isn't configured
    private IntakeState state = IntakeState.OFF;

    public Intake(HardwareMap hw) {
        motor = new CachedMotor(hw, HardwareNames.INTAKE);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        if (hw.tryGet(DcMotorEx.class, HardwareNames.TRANSFER) != null) {
            transferMotor = new CachedMotor(hw, HardwareNames.TRANSFER);
            transferMotor.setDirection(TRANSFER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        } else {
            transferMotor = null;
        }
    }

    private void setPower(double power) {
        motor.setPower(power);
        if (transferMotor != null) transferMotor.setPower(power);
    }

    public void on() {
        setPower(ON_POWER);
        state = IntakeState.ON;
    }

    public void reverse() {
        setPower(REVERSE_POWER);
        state = IntakeState.REVERSE;
    }

    public void slowReverse() {
        setPower(SLOW_REVERSE_POWER);
        state = IntakeState.SLOW_REVERSE;
    }

    public void off() {
        setPower(0);
        state = IntakeState.OFF;
    }

    public IntakeState getState() {
        return state;
    }

    public boolean atState(IntakeState state) {
        return state == this.state;
    }

    public boolean hasTransferMotor() {
        return transferMotor != null;
    }
}
