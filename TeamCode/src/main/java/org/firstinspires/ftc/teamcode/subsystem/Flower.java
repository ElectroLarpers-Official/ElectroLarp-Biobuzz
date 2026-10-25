package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

public class Flower {
    public static double OPEN_POS = 1.0;
    public static double CLOSED_POS = 0;

    private final ServoImplEx flower; // null if flowerServo isn't in the config: everything is a no-op

    public Flower(HardwareMap hw) {
        flower = hw.tryGet(ServoImplEx.class, HardwareNames.FLOWER_SERVO);
    }

    public void toggleFlower(boolean state) {
        if (flower != null) flower.setPosition(state ? OPEN_POS : CLOSED_POS);
    }

    public boolean isConnected() { return flower != null; }
}
