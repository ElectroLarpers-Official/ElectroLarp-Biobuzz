package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Two-position latch servo. Starts closed and holds whichever position it was last set to. */
@Configurable
public final class Latch {
    public static String SERVO_NAME = "stopperServo";
    public static double OPEN_POS = 0.6;
    public static double CLOSED_POS = 0.0;

    private final Servo latch;
    private boolean open;

    public Latch(HardwareMap hw) {
        latch = hw.get(Servo.class, SERVO_NAME);
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
