package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

/** Single-motor intake with three states: intake, outtake, stopped. */
@Configurable
public final class Intake {
    public static String MOTOR_NAME = "intakeMotor";
    public static boolean REVERSED = false;

    public static double INTAKE = 1.0;
    public static double OUTTAKE = -1.0;
    public static double STOPPED = 0;



    public enum IntakeState {
        INTAKE,
        OUTTAKE,

        STOPPED
    }

    private final CachedMotor motor;
    private IntakeState state = IntakeState.STOPPED;

    public Intake(HardwareMap hw) {
        motor = new CachedMotor(hw, MOTOR_NAME);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void intake() {
        motor.setPower(INTAKE);
        state = IntakeState.INTAKE;
    }

    public void outtake() {
        motor.setPower(OUTTAKE);
        state = IntakeState.OUTTAKE;
    }

    public void stop() {
        motor.setPower(STOPPED);
        state = IntakeState.STOPPED;
    }

    public IntakeState getState() {
        return state;
    }

    public boolean atState(IntakeState state) {
        return state == this.state;
    }
}
