package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.HardwareNames;

@Configurable
public class Flower {
    public static double DOWN_POS = 0.75;
    public static double UP_POS = 0.11;

    private final ServoImplEx flower;
    private boolean down = false;

    public Flower(HardwareMap hw) {
        flower = hw.tryGet(ServoImplEx.class, HardwareNames.FLOWER_SERVO);
    }

    public void setDown(boolean state) {
        down = state;
        if (flower != null) flower.setPosition(state ? DOWN_POS : UP_POS);
    }

    public void update() {}

    public void stop() {}

    public void toggle() { setDown(!down); }

    public boolean isDown() { return down; }

    public boolean isConnected() { return flower != null; }
}
