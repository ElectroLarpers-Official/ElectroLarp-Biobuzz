package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.subsystem.aim.shot.ShooterCalc;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

/**
 * Puts the hood at a launch angle so you can check it with a protractor and tune the servo calibration.
 * Starts at the flattest angle (43.01765 deg = servo position 0). Needs only hoodServo.
 *
 * <p>Gamepad 1: dpad up / down = +/- 1 deg, dpad right / left = +/- 0.1 deg, A = back to 43.01765, Y = 80.
 * TARGET_LAUNCH_DEG is also editable in Panels. If 43.02 deg isn't exactly on the flat stop, adjust
 * ShooterCalc.HOOD_SERVO_POS_AT_LAUNCH_MIN; if 80 deg is off, check HOOD_SERVO_RANGE_DEG (300 for goBILDA).
 */
@Configurable
@TeleOp(name = "HoodServoPositionTuning", group = "Shooter Test")
public class HoodServoPositionTuning extends CommandOpMode {
    public static double TARGET_LAUNCH_DEG = 43.01765;

    Hood hood;
    GamepadEx gp1;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        hood = new Hood(hardwareMap);
        TARGET_LAUNCH_DEG = ShooterCalc.LAUNCH_MIN_DEG;
        hood.setLaunchAngle(TARGET_LAUNCH_DEG);   // goes straight to 43.02 deg at INIT
    }

    @Override
    public void initializeLoop() {
        hood.setLaunchAngle(TARGET_LAUNCH_DEG);
        show();
    }

    @Override
    public void execute() {
        gp1.update();
        if (gp1.dpad_up.isRisingEdge()) TARGET_LAUNCH_DEG += 1.0;
        if (gp1.dpad_down.isRisingEdge()) TARGET_LAUNCH_DEG -= 1.0;
        if (gp1.dpad_right.isRisingEdge()) TARGET_LAUNCH_DEG += 0.1;
        if (gp1.dpad_left.isRisingEdge()) TARGET_LAUNCH_DEG -= 0.1;
        if (gp1.a.isRisingEdge()) TARGET_LAUNCH_DEG = ShooterCalc.LAUNCH_MIN_DEG;
        if (gp1.y.isRisingEdge()) TARGET_LAUNCH_DEG = ShooterCalc.LAUNCH_MAX_DEG;
        TARGET_LAUNCH_DEG = Math.max(ShooterCalc.LAUNCH_MIN_DEG, Math.min(ShooterCalc.LAUNCH_MAX_DEG, TARGET_LAUNCH_DEG));
        hood.setLaunchAngle(TARGET_LAUNCH_DEG);
        show();
    }

    private void show() {
        telemetry.addLine("dpad up/down +/-1 deg, right/left +/-0.1 deg, A = 43.02, Y = 80");
        telemetry.addData("Launch angle (deg)", "%.2f", TARGET_LAUNCH_DEG);
        telemetry.addData("Servo position", "%.4f", hood.position());
        telemetry.update();
    }
}
