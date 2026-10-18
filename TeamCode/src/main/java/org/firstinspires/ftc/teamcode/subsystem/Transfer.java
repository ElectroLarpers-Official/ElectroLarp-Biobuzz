// DISABLED: the transfer is belted to the intake on the robot, so Intake now drives both motors.
// Kept for reference; delete once you're happy with Intake.
/*
package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.util.hardware.CachedMotor;

@Configurable
public final class Transfer {
    public static boolean MOTOR_1_REVERSED = false; // TODO: check direction on the new robot
    public static boolean MOTOR_2_REVERSED = false; // TODO: check direction on the new robot

    public static double INTAKE = 1.0;
    public static double OUTTAKE = -1.0;
    public static double SLOW = -0.5; // slow reverse, used to clear a jam
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
        motor1 = new CachedMotor(hw, HardwareNames.TRANSFER_1);
        motor2 = new CachedMotor(hw, HardwareNames.TRANSFER_2);
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
*/
