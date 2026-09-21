package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Carries state from one OpMode to the next. Static fields survive between OpModes as long as the robot
 * controller app keeps running, which is how the autonomous hands its final pose to TeleOp.
 */
public final class OpModeStorage {
    private OpModeStorage() {}

    /** Where the last autonomous (or TeleOp) run left the robot, or null if nothing has run yet. */
    public static Pose endPose = null;
}
