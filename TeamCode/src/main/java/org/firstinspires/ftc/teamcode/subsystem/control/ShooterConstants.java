package org.firstinspires.ftc.teamcode.subsystem.control;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class ShooterConstants {
    // TODO: set both to the BIOBUZZ target's field position (Pedro inches). These are carried over from the old code.
    public static Pose GOAL_POS_RED = new Pose(144, 144);
    public static Pose GOAL_POS_BLUE = new Pose(0, 144);

    // TODO: recalibrate for the BIOBUZZ target (carried over from the old code).
    public static double SCORE_ANGLE = -17;

    // TODO: measure for the BIOBUZZ target.
    public static double PASS_THROUGH_POINT_RADIUS = 0; //inches change to 4 if goon

    public static double RAISE_TIME = 0.15;

}
