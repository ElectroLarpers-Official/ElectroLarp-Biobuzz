package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

@Configurable
public final class Intake {
    public static boolean REVERSED = false; // TODO: check direction on the new robot

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
    private IntakeState state = IntakeState.STOPPED;

    public Intake(HardwareMap hw) {
        motor = new CachedMotor(hw, HardwareNames.INTAKE);
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

    public void slow() {
        motor.setPower(SLOW);
        state = IntakeState.SLOW;
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
