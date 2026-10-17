package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.util.Alliance;

public final class ClosePaths {
    private ClosePaths() {}

    private static final double MIRROR_LINE = 70.75;

    public static PoseFactory poses(Alliance a) {
        PoseFactory factory = PoseFactory.degrees();
        return a == Alliance.BLUE ? factory.mirrorX(MIRROR_LINE) : factory;
    }

    public static Pose startPose(Alliance a) {
        return new Poses(a).start;
    }

    public static Path path1(Alliance a) {
        Poses waypoints = new Poses(a);
        return curve(waypoints.start, waypoints.path1Control1, waypoints.path1Control2, waypoints.path1Control3, waypoints.path1)
                .heading(Interpolator.piecewise()
                        .until(0.6096, Interpolator.tangent)
                        .until(1, Interpolator.constant(waypoints.path1End)));
    }

    public static Path path2(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.path1, waypoints.point2)
                .heading(Interpolator.piecewise()
                        .until(0.3865, Interpolator.linear(waypoints.point2Start, waypoints.point2Mid))
                        .until(1, Interpolator.constant(waypoints.point2)));
    }

    public static Path path3(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.point2, waypoints.point3).constant(waypoints.point3);
    }

    public static Path path4(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.point3, waypoints.point4).constant(waypoints.point4);
    }

    public static Path path5(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.point4, waypoints.point5).constant(waypoints.point5);
    }

    public static Path path6(Alliance a) {
        Poses waypoints = new Poses(a);
        return line(waypoints.point5, waypoints.point6).constant(waypoints.point6);
    }

    private static final class Poses {
        final Pose start;

        final Pose path1;
        final Pose path1End;
        final Pose path1Control1;
        final Pose path1Control2;
        final Pose path1Control3;

        final Pose point2;
        final Pose point2Start;
        final Pose point2Mid;

        final Pose point3;
        final Pose point4;
        final Pose point5;
        final Pose point6;

        Poses(Alliance a) {
            PoseFactory poseFactory = poses(a);

            start = poseFactory.of(32.52, 8.824, 90);

            path1 = poseFactory.of(48.1732, 119.417, 90);
            path1End = poseFactory.of(48.1732, 119.417, 90);
            path1Control1 = poseFactory.of(30.8661, 30.9884, 0);
            path1Control2 = poseFactory.of(11.3969, 92.0999, 0);
            path1Control3 = poseFactory.of(52.2656, 109.441, 0);

            point2 = poseFactory.of(9.4665, 106.155, 180);
            point2Start = poseFactory.of(9.4665, 106.155, 90);
            point2Mid = poseFactory.of(9.4665, 106.155, 180);

            point3 = poseFactory.of(26.4127, 48.6033, 180);
            point4 = poseFactory.of(20.508, 48.5437, 180);
            point5 = poseFactory.of(21.5138, 8.4811, 180);
            point6 = poseFactory.of(9.714, 8.4068, 180);
        }
    }
}
