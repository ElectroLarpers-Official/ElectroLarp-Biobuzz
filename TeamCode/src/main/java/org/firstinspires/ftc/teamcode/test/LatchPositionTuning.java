package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.Latch;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

/**
 * Tunes Latch.OPEN_POS / Latch.CLOSED_POS. Needs only the latch servo (HardwareNames.LATCH).
 * Drives the servo directly, so it can go to any position (Latch itself only knows open/closed).
 * Nothing moves until you press a button after START.
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li>A = go to TARGET_POS (type it in Panels, then press A)</li>
 *   <li>dpad up / down = target +/- 0.01, right / left = +/- 0.001 (moves right away)</li>
 *   <li>X = go to Latch.OPEN_POS, B = go to Latch.CLOSED_POS, Y = toggle between them (cycle test)</li>
 *   <li>left bumper = save current position as Latch.OPEN_POS, right bumper = save as Latch.CLOSED_POS</li>
 * </ul>
 * LIVE_FOLLOW = true makes the servo follow TARGET_POS every loop (no A needed).
 * Saved values only live until restart / Sloth reload: copy them into Latch.java.
 */
@Configurable
@TeleOp(name = "LatchPositionTuning", group = "Test")
public class LatchPositionTuning extends CommandOpMode {
    public static double TARGET_POS = 0.0;
    public static boolean LIVE_FOLLOW = false;
    public static double COARSE_STEP = 0.01;
    public static double FINE_STEP = 0.001;

    private Servo servo;
    private GamepadEx gp1;
    private TelemetryManager panels;

    private double commanded = Double.NaN; // NaN = never commanded
    private String lastAction = "none";

    @Override
    public void initialize() {
        gp1 = new GamepadEx(gamepad1);
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        servo = hardwareMap.tryGet(Servo.class, HardwareNames.LATCH);
        TARGET_POS = Latch.CLOSED_POS;
    }

    @Override
    public void initializeLoop() {
        gp1.update(); // so a button held through START isn't read as a press
        show();
    }

    @Override
    public void execute() {
        gp1.update();

        if (gp1.dpad_up.isRisingEdge()) nudge(COARSE_STEP);
        if (gp1.dpad_down.isRisingEdge()) nudge(-COARSE_STEP);
        if (gp1.dpad_right.isRisingEdge()) nudge(FINE_STEP);
        if (gp1.dpad_left.isRisingEdge()) nudge(-FINE_STEP);

        if (gp1.a.isRisingEdge()) moveTo(TARGET_POS, "A: target");
        if (gp1.x.isRisingEdge()) moveTo(Latch.OPEN_POS, "X: OPEN_POS");
        if (gp1.b.isRisingEdge()) moveTo(Latch.CLOSED_POS, "B: CLOSED_POS");
        if (gp1.y.isRisingEdge()) {
            boolean atOpen = commanded == Latch.OPEN_POS;
            if (atOpen) moveTo(Latch.CLOSED_POS, "Y: toggle -> closed");
            else moveTo(Latch.OPEN_POS, "Y: toggle -> open");
        }

        if (gp1.left_bumper.isRisingEdge()) save(true);
        if (gp1.right_bumper.isRisingEdge()) save(false);

        if (LIVE_FOLLOW && clamp(TARGET_POS) != commanded) moveTo(TARGET_POS, "live follow");

        show();
    }

    private void nudge(double step) {
        TARGET_POS = clamp(TARGET_POS + step);
        moveTo(TARGET_POS, String.format("nudge %+.3f", step));
    }

    private void moveTo(double pos, String why) {
        commanded = clamp(pos);
        if (servo != null) servo.setPosition(commanded);
        lastAction = why;
    }

    private void save(boolean asOpen) {
        if (Double.isNaN(commanded)) {
            lastAction = "nothing to save: move the servo first";
            return;
        }
        if (asOpen) Latch.OPEN_POS = commanded;
        else Latch.CLOSED_POS = commanded;
        PanelsConfigurables.refreshClass(Latch.class); // show the new value in Panels' Latch group
        lastAction = String.format("saved %.4f as %s", commanded, asOpen ? "OPEN_POS" : "CLOSED_POS");
    }

    private static double clamp(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    private void show() {
        if (servo == null) panels.addLine("!! No servo named \"" + HardwareNames.LATCH + "\" in the config");
        panels.addLine("A = go to target | dpad = nudge | X open, B closed, Y toggle");
        panels.addLine("LB = save as OPEN_POS, RB = save as CLOSED_POS (copy into Latch.java!)");
        panels.addData("Target pos", String.format("%.4f", TARGET_POS));
        panels.addData("Commanded pos", Double.isNaN(commanded) ? "none" : String.format("%.4f", commanded));
        panels.addData("Latch.OPEN_POS", String.format("%.4f", Latch.OPEN_POS));
        panels.addData("Latch.CLOSED_POS", String.format("%.4f", Latch.CLOSED_POS));
        panels.addData("Live follow", LIVE_FOLLOW);
        panels.addData("Last action", lastAction);
        panels.update(telemetry); // Driver Station + Panels
    }
}
