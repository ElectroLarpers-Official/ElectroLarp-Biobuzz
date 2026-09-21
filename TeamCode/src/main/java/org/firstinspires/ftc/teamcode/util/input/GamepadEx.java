package org.firstinspires.ftc.teamcode.util.input;

import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * Wraps an SDK {@link Gamepad} with edge/toggle-aware buttons and cached analog axes.
 * Call {@link #update()} once per loop, before anything reads it (see {@code CommandOpMode#execute}).
 */
public class GamepadEx {
    public final BooleanSwitch b;
    public final BooleanSwitch a;
    public final BooleanSwitch x;
    public final BooleanSwitch y;
    public final BooleanSwitch left_bumper;
    public final BooleanSwitch right_bumper;
    public final BooleanSwitch start;
    public final BooleanSwitch back;
    public final BooleanSwitch dpad_left;
    public final BooleanSwitch dpad_up;
    public final BooleanSwitch dpad_down;
    public final BooleanSwitch dpad_right;
    public final BooleanSwitch left_stick_button;
    public final BooleanSwitch right_stick_button;
    public final BooleanSwitch touchpad;

    /** Insertion-ordered so buttons update in a deterministic order. */
    public Set<BooleanSwitch> buttons;

    public final FloatSupplier right_trigger;
    public final FloatSupplier left_trigger;
    public final FloatSupplier right_stick_y;
    public final FloatSupplier right_stick_x;
    public final FloatSupplier left_stick_y;
    public final FloatSupplier left_stick_x;

    public BooleanSwitch right_trigger_button;
    public BooleanSwitch left_trigger_button;
    public BooleanSwitch left_stick_x_button;
    public BooleanSwitch left_stick_y_button;
    public BooleanSwitch right_stick_x_button;
    public BooleanSwitch right_stick_y_button;

    private final Gamepad gamepad;
    private final FloatSupplier[] axes;

    public GamepadEx(Gamepad gamepad) {
        a = new BooleanSwitch(() -> gamepad.a);
        b = new BooleanSwitch(() -> gamepad.b);
        x = new BooleanSwitch(() -> gamepad.x);
        y = new BooleanSwitch(() -> gamepad.y);
        left_bumper = new BooleanSwitch(() -> gamepad.left_bumper);
        right_bumper = new BooleanSwitch(() -> gamepad.right_bumper);
        start = new BooleanSwitch(() -> gamepad.start);
        back = new BooleanSwitch(() -> gamepad.back);
        dpad_left = new BooleanSwitch(() -> gamepad.dpad_left);
        dpad_right = new BooleanSwitch(() -> gamepad.dpad_right);
        dpad_down = new BooleanSwitch(() -> gamepad.dpad_down);
        dpad_up = new BooleanSwitch(() -> gamepad.dpad_up);
        left_stick_button = new BooleanSwitch(() -> gamepad.left_stick_button);
        right_stick_button = new BooleanSwitch(() -> gamepad.right_stick_button);
        touchpad = new BooleanSwitch(() -> gamepad.touchpad);
        right_trigger = new FloatSupplier(() -> gamepad.right_trigger);
        left_trigger = new FloatSupplier(() -> gamepad.left_trigger);
        right_stick_x = new FloatSupplier(() -> gamepad.right_stick_x);
        left_stick_x = new FloatSupplier(() -> gamepad.left_stick_x);
        right_stick_y = new FloatSupplier(() -> gamepad.right_stick_y);
        left_stick_y = new FloatSupplier(() -> gamepad.left_stick_y);
        // Arrays.asList, not List.of: minSdk is 24 and List.of needs API 30.
        buttons = new LinkedHashSet<>(Arrays.asList(
                a, b, x, y, left_bumper, right_bumper, start, back, dpad_left, dpad_right, dpad_down, dpad_up, left_stick_button,
                right_stick_button, touchpad
        ));
        axes = new FloatSupplier[]{
                right_trigger, left_trigger, right_stick_y, right_stick_x, left_stick_y, left_stick_x
        };
        this.gamepad = gamepad;
    }

    public void apply(Function<BooleanSwitch, BooleanSwitch> function) {
        buttons = buttons.stream().map(function).collect(Collectors.toCollection(LinkedHashSet::new));
    }

    public void left_trigger_button(Function<FloatSupplier, BooleanSwitch> f) {
        left_trigger_button = f.apply(left_trigger);
        buttons.add(left_trigger_button);
    }

    public void right_trigger_button(Function<FloatSupplier, BooleanSwitch> f) {
        right_trigger_button = f.apply(right_trigger);
        buttons.add(right_trigger_button);
    }

    public void left_stick_x_button(Function<FloatSupplier, BooleanSwitch> f) {
        left_stick_x_button = f.apply(left_stick_x);
        buttons.add(left_stick_x_button);
    }

    public void left_stick_y_button(Function<FloatSupplier, BooleanSwitch> f) {
        left_stick_y_button = f.apply(left_stick_y);
        buttons.add(left_stick_y_button);
    }

    public void right_stick_x_button(Function<FloatSupplier, BooleanSwitch> f) {
        right_stick_x_button = f.apply(right_stick_x);
        buttons.add(right_stick_x_button);
    }

    public void right_stick_y_button(Function<FloatSupplier, BooleanSwitch> f) {
        right_stick_y_button = f.apply(right_stick_y);
        buttons.add(right_stick_y_button);
    }

    /** Stick as (right, up), so pushing the stick forward is +y. */
    public Vector2D leftStick() {
        return Vector2D.cartesian(gamepad.left_stick_x, -gamepad.left_stick_y);
    }

    /** Stick as (right, up), so pushing the stick forward is +y. */
    public Vector2D rightStick() {
        return Vector2D.cartesian(gamepad.right_stick_x, -gamepad.right_stick_y);
    }

    public void rumble(double ms) {
        gamepad.rumble((int) ms);
    }

    public void addButton(BooleanSwitch button) {
        buttons.add(button);
    }

    public void update() {
        // The raw axes are only refreshed here; without this getVal() would stay 0 unless a derived button used them.
        for (FloatSupplier axis : axes) axis.update();
        for (BooleanSwitch button : buttons) button.update();
    }
}
