package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public final class Latch {
    public static double OPEN_POS = 0; // TODO(new robot): old-robot value
    public static double CLOSED_POS = 0.142; // TODO(new robot): old-robot value

    private final Servo latch;
    private boolean open;

    public Latch(HardwareMap hw) {
        latch = hw.get(Servo.class, HardwareNames.LATCH);
        latch.setPosition(CLOSED_POS);
        open = false;
    }

    public void open() {
        if (open) return;
        open = true;
        latch.setPosition(OPEN_POS);
    }

    public void close() {
        if (!open) return;
        open = false;
        latch.setPosition(CLOSED_POS);
    }

    public boolean isOpen() { return open; }
}
