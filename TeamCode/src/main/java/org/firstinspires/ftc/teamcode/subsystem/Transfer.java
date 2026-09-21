package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

/** Two-motor transfer, bang-bang (fixed power per state) with states: intake, slow, outtake, stopped. */
@Configurable
public final class Transfer {
    public static String MOTOR_1_NAME = "transferMotor";
    public static String MOTOR_2_NAME = "transferMotor2";
    public static boolean MOTOR_1_REVERSED = false;
    public static boolean MOTOR_2_REVERSED = false;

    public static double INTAKE = 1.0;
    public static double OUTTAKE = -1.0;
    public static double SLOW = 0.8;
    public static double STOPPED = 0;

    public enum TransferState {
        INTAKE,
        OUTTAKE,
        SLOW,
        STOPPED
    }

    private final CachedMotor motor1, motor2;
    private TransferState state = TransferState.STOPPED;

    public Transfer(HardwareMap hw) {
        motor1 = new CachedMotor(hw, MOTOR_1_NAME);
        motor2 = new CachedMotor(hw, MOTOR_2_NAME);
        motor1.setDirection(MOTOR_1_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(MOTOR_2_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    private void setPower(double power) {
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void intake() {
        setPower(INTAKE);
        state = TransferState.INTAKE;
    }

    public void slow() {
        setPower(SLOW);
        state = TransferState.SLOW;
    }

    public void outtake() {
        setPower(OUTTAKE);
        state = TransferState.OUTTAKE;
    }

    public void stop() {
        setPower(STOPPED);
        state = TransferState.STOPPED;
    }

    public TransferState getState() {
        return state;
    }

    public boolean atState(TransferState state) {
        return state == this.state;
    }
}
