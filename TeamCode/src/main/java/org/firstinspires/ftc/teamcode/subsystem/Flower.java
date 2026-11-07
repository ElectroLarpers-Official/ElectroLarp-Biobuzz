package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

public class Flower {
    public static double DOWN_POS = 0.25;
    public static double UP_POS = 1.0;

    private final ServoImplEx flower;
    private boolean down = false;

    public Flower(HardwareMap hw) {
        flower = hw.tryGet(ServoImplEx.class, HardwareNames.FLOWER_SERVO);
    }

    public void setDown(boolean state) {
        down = state;
        if (flower != null) flower.setPosition(state ? DOWN_POS : UP_POS);
    }

    public void toggle() { setDown(!down); }

    public boolean isDown() { return down; }

    public boolean isConnected() { return flower != null; }
}
