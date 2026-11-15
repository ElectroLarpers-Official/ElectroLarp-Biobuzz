package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public class Flower {
    public static double DOWN_POWER = 1.0;
    public static double UP_POWER = -1.0;
    public static double RUN_MS = 500;

    private final CRServo flower;
    private boolean down = false;
    private long stopAtNanos = 0;

    public Flower(HardwareMap hw) {
        CRServo found = hw.tryGet(CRServoImplEx.class, HardwareNames.FLOWER_SERVO);
        if (found == null) found = hw.tryGet(CRServo.class, HardwareNames.FLOWER_SERVO);
        flower = found;
        if (flower instanceof CRServoImplEx) ((CRServoImplEx) flower).setPwmEnable();
    }

    public void setDown(boolean state) {
        if (flower == null) return;
        down = state;
        flower.setPower(state ? DOWN_POWER : UP_POWER);
        stopAtNanos = System.nanoTime() + (long) (RUN_MS * 1e6);
    }

    public void update() {
        if (flower == null || stopAtNanos == 0) return;
        if (System.nanoTime() >= stopAtNanos) {
            flower.setPower(0);
            stopAtNanos = 0;
        }
    }

    public void stop() {
        if (flower != null) flower.setPower(0);
        stopAtNanos = 0;
    }

    public void toggle() { setDown(!down); }

    public boolean isDown() { return down; }

    public boolean isConnected() { return flower != null; }
}
