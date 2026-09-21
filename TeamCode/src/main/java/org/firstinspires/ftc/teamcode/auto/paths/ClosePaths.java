package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Alliance;

/** Paths and poses for the close-side auto. Written once for BLUE; RED is mirrored by the pose factory. */
public final class ClosePaths {
    private ClosePaths() {}

    /** TODO: confirm the mirror axis and field size against the BIOBUZZ field and Pedro's coordinate system. */
    private static final double MIRROR_LINE = 72;

    /** Every pose made by this factory is already mirrored for {@code a}. */
    public static PoseFactory poses(Alliance a) {
        PoseFactory factory = PoseFactory.degrees();
        return a == Alliance.RED ? factory.mirrorX(MIRROR_LINE) : factory;
    }

    /** TODO: placeholder, measure the real starting pose (inches, degrees). */
    public static Pose startPose(Alliance a) {
        return poses(a).of(0, 0, 0);
    }
}
