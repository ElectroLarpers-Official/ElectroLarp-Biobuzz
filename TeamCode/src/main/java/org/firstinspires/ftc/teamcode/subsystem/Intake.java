package org.firstinspires.ftc.teamcode.subsystem;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
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

    public static double INTAKE = 1.0;
    public static double OUTTAKE = -1.0;
    public static double SLOW = -0.5; // slow reverse, used to clear a jam
    public static double STOPPED = 0;

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        SLOW,
        STOPPED
    }

    private final CachedMotor motor;
    private final CachedMotor transferMotor; // null if "transfer" isn't configured
    private IntakeState state = IntakeState.STOPPED;

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

    public Command intake() {
        return instant(() -> {
            setPower(INTAKE);
            state = IntakeState.INTAKE;
        });
    }

    public Command outtake() {
        return instant(() -> {
            setPower(OUTTAKE);
            state = IntakeState.OUTTAKE;
        });
    }

    public Command slow() {
        return instant(() -> {
            setPower(SLOW);
            state = IntakeState.SLOW;
        });
    }

    public Command stop() {
        return instant(() -> {
            setPower(STOPPED);
            state = IntakeState.STOPPED;
        });
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
