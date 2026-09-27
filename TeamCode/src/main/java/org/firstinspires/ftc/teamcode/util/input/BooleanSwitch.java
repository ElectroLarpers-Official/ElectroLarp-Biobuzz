package org.firstinspires.ftc.teamcode.util.input;

import java.util.function.BooleanSupplier;

public final class BooleanSwitch {
    private boolean state;
    private boolean previousState;
    private boolean toggled;
    private final BooleanSupplier decider;
    private Runnable updateFunction;
    private long lastTrueTime;
    public static final int BTN_PRESS_INTERVAL = 250; // milliseconds

    public BooleanSwitch(BooleanSupplier decider) {
        this.decider = decider;
    }

    public void update() {
        if (updateFunction != null)
            updateFunction.run();
        previousState = state;
        state = decider.getAsBoolean();

        if (isRisingEdge()) {
            toggled = !toggled;
        }
    }

    public boolean isTrue() {
        return state;
    }

    public boolean isRisingEdge() {
        return state && !previousState;
    }

    public boolean isFallingEdge() {
        return !state && previousState;
    }

    public boolean isToggled() {
        return toggled;
    }

    public BooleanSwitch risingEdge() {
        return new BooleanSwitch(this::isRisingEdge).setUpdateFunction(this::update);
    }

    public BooleanSwitch fallingEdge() {
        return new BooleanSwitch(this::isFallingEdge).setUpdateFunction(this::update);
    }

    public BooleanSwitch toggled() {
        return new BooleanSwitch(this::isToggled).setUpdateFunction(this::update);
    }

    public BooleanSwitch and(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state && other.state).setUpdateFunction(() -> {
            update();
            other.update();
        });
    }

    public BooleanSwitch and(BooleanSupplier other) {
        return and(new BooleanSwitch(other));
    }

    public BooleanSwitch or(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state || other.state).setUpdateFunction(() -> {
            update();
            other.update();
        });
    }

    public BooleanSwitch or(BooleanSupplier other) {
        return or(new BooleanSwitch(other));
    }

    public BooleanSwitch not() {
        return new BooleanSwitch(() -> !this.state);
    }

    private boolean canPress(long timestamp, long interval) {
        if (!isTrue()) return false;
        boolean isTrue = (timestamp - lastTrueTime) > interval;
        if (isTrue) lastTrueTime = timestamp;
        return isTrue;
    }

    public boolean canPress(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL);
    }

    public boolean canPressShort(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/2);
    }

    public boolean canPress4Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/4);
    }

    public boolean canPress6Short(long timeStamp) {
        return canPress(timeStamp,
                BTN_PRESS_INTERVAL/6);
    }

    public BooleanSwitch setUpdateFunction(Runnable r) {
        this.updateFunction = r;
        return this;
    }
}
