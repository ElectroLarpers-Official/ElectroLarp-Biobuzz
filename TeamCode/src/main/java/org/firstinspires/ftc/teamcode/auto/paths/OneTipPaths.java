package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.util.Alliance;

public final class OneTipPaths {
    private OneTipPaths() {}

    private static final double MIRROR_LINE = 70.75;

    /** Blue mirrors y about this line too (x about MIRROR_LINE): the blue path is the red one turned 180 deg. */
    private static final double MIRROR_LINE_Y = 72;

    public static PoseFactory poses(Alliance a) {
        PoseFactory factory = PoseFactory.degrees();
        return a == Alliance.BLUE ? factory.mirrorX(MIRROR_LINE) : factory;
    }

    /** Red pose (inches, degrees) -> this alliance's pose. Blue: x and y both mirrored, heading + 180. */
    private static Pose of(Alliance a, double x, double y, double headingDeg) {
        if (a != Alliance.BLUE) return new Pose(x, y, Math.toRadians(headingDeg));
        return new Pose(2 * MIRROR_LINE - x, 2 * MIRROR_LINE_Y - y, Math.toRadians(headingDeg + 180));
    }

    public static Pose startPose(Alliance a) {
        return new Poses(a).start;
    }

    public static Path path1(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.start, waypoints.path1).constant(waypoints.path1);
    }

    public static Path path2(Alliance a) {
        Poses waypoints = new Poses(a);
        return curve(waypoints.path1, waypoints.point2Control1, waypoints.point2Control2, waypoints.point2)
                .constant(waypoints.point2);
    }

    private static final class Poses {
        final Pose start;
        final Pose path1;
        final Pose point2;
        final Pose point2Control1;
        final Pose point2Control2;

        Poses(Alliance a) {
            start = of(a, 63.8268, 9.4418, 90);
            path1 = of(a, 55, 33, 90);
            point2 = of(a, 7.8392, 124.8472, 90);
            point2Control1 = of(a, 17.147, 29.0335, 0);
            point2Control2 = of(a, 44.7977, 129.5429, 0);
        }
    }
}
