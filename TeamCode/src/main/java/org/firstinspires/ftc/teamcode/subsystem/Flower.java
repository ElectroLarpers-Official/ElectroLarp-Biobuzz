package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

public class Flower {
    public static double DOWN_POS = 0.25;
    public static double UP_POS = 1.0;

    private final ServoImplEx flower; // null if flowerServo isn't in the config: everything is a no-op
    private boolean down = false;

    public Flower(HardwareMap hw) {
        flower = hw.tryGet(ServoImplEx.class, HardwareNames.FLOWER_SERVO);
    }

    /** true = flower down (DOWN_POS), false = flower up (UP_POS). */
    public void setDown(boolean state) {
        down = state;
        if (flower != null) flower.setPosition(state ? DOWN_POS : UP_POS);
    }

    /** Puts the flower down if it is up, up if it is down. */
    public void toggle() { setDown(!down); }

    public boolean isDown() { return down; }

    public boolean isConnected() { return flower != null; }
}
