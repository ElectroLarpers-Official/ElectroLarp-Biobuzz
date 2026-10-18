package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public final class Latch {
    public static double OPEN_POS = 0; // TODO(new robot): old-robot value
    public static double CLOSED_POS = 0.142; // TODO(new robot): old-robot value

    private final Servo latch; // null if stopperServo isn't in the config (not plugged in yet): everything is a no-op
    private boolean open;

    public Latch(HardwareMap hw) {
        latch = hw.tryGet(Servo.class, HardwareNames.LATCH);
        if (latch != null) latch.setPosition(CLOSED_POS);
        open = false;
    }

    public void open() {
        if (open) return;
        open = true;
        if (latch != null) latch.setPosition(OPEN_POS);
    }

    public void close() {
        if (!open) return;
        open = false;
        if (latch != null) latch.setPosition(CLOSED_POS);
    }

    public boolean isOpen() { return open; }
    public boolean isConnected() { return latch != null; }
}
