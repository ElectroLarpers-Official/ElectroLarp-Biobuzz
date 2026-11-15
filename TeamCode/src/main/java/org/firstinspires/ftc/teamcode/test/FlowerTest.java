package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.Flower;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;


@Configurable
@TeleOp(name = "FlowerTest", group = "Test")
public class FlowerTest extends CommandOpMode {
    private CRServo servo;
    private Flower flower;
    private GamepadEx gp1;
    private TelemetryManager panels;

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        servo = hardwareMap.tryGet(CRServo.class, HardwareNames.FLOWER_SERVO);
        flower = new Flower(hardwareMap);
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        show();
    }

    @Override
    public void execute() {
        gp1.update();
        if (gp1.a.isRisingEdge()) flower.setDown(true);
        if (gp1.y.isRisingEdge()) flower.setDown(false);
        if (gp1.b.isRisingEdge()) flower.stop();
        flower.update();
        show();
    }

    @Override
    public void end() {
        if (flower != null) flower.stop();
    }

    private void show() {
        if (servo == null) panels.addLine("!! No continuous-rotation servo named \"" + HardwareNames.FLOWER_SERVO
                + "\" in the config (set the device type to Continuous Rotation Servo)");
        panels.addLine("A = down | Y = up | B = stop");
        panels.addData("Down", flower != null && flower.isDown());
        panels.addData("Flower.DOWN_POWER", Flower.DOWN_POWER);
        panels.addData("Flower.UP_POWER", Flower.UP_POWER);
        panels.addData("Flower.RUN_MS", Flower.RUN_MS);
        panels.update(telemetry);
    }
}
