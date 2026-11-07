package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.Flower;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;


@Configurable
@TeleOp(name = "FlowerTest", group = "Test")
public class FlowerTest extends CommandOpMode {
    public static double STEP = 0.1;

    private Servo servo;
    private GamepadEx gp1;
    private TelemetryManager panels;

    private double pos = Flower.DOWN_POS;
    private boolean commanded = false;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        servo = hardwareMap.tryGet(Servo.class, HardwareNames.FLOWER_SERVO);
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        show();
    }

    @Override
    public void execute() {
        gp1.update();

        if (gp1.dpad_up.isRisingEdge()) moveTo(pos + STEP);
        if (gp1.dpad_down.isRisingEdge()) moveTo(pos - STEP);
        if (gp1.a.isRisingEdge()) moveTo(commanded && pos == Flower.DOWN_POS ? Flower.UP_POS : Flower.DOWN_POS);

        show();
    }

    private void moveTo(double target) {
        pos = Math.round(Math.max(0.0, Math.min(1.0, target)) * 1000.0) / 1000.0;
        commanded = true;
        if (servo != null) servo.setPosition(pos);
    }

    private void show() {
        if (servo == null) panels.addLine("!! No servo named \"" + HardwareNames.FLOWER_SERVO + "\" in the config");
        panels.addLine("dpad up / down = +/- " + STEP + " | A = toggle down / up");
        panels.addData("Position", commanded ? String.format("%.3f", pos) : "not moved yet");
        panels.addData("Flower.DOWN_POS", Flower.DOWN_POS);
        panels.addData("Flower.UP_POS", Flower.UP_POS);
        panels.update(telemetry);
    }
}
