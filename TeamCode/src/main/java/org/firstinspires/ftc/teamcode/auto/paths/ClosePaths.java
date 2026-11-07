package org.firstinspires.ftc.teamcode.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import org.electrolightsftc.pulse.lumen.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import org.electrolightsftc.pulse.electropathing.*;
import static org.electrolightsftc.pulse.electropathing.PiecewiseCustom.*;
import static org.electrolightsftc.pulse.electropathing.AutoPathRuntime.*;
import static org.electrolightsftc.pulse.electropathing.PulseRuntime.follower;
import static org.electrolightsftc.pulse.electropathing.PulseFollower.*;
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

    //Real Code:

    public static Path intakeLoading(Alliance a){
        Poses waypoints = new Poses(a);
        return curve(waypoints.start, waypoints.path1Control1, waypoints.pollenEndingPose)
                .heading(Interpolator.piecewise()
                        .until(0.2324, Interpolator.constant(waypoints.path1Segment1Heading))
                        .until(1, Interpolator.constant(waypoints.path1Segment2Heading)));
    }

    public static Path goNearFlower(Alliance a, Command openFlower) {
        Poses waypoints = new Poses(a);
        return follower().pathBuilder()
                .addPath(curve(waypoints.pollenEndingPose, waypoints.cpFlower, waypoints.endPointFlower)
                        .heading(Interpolator.piecewise()
                                .until(0.1687, Interpolator.linear(waypoints.point2Segment1Start, waypoints.point2Segment1End))
                                .until(1, Interpolator.tangent)))
                .addPoseCallback(waypoints.flowerTrigger, openFlower, 1.0)
                .build();
    }

    public static Path goInFlower(Alliance a) {
        Poses waypoints = new Poses(a);
        return curve(waypoints.endPointFlower, waypoints.point3Control1, waypoints.point3).constant(waypoints.point3);
    }

    public static Path park(Alliance a) {
        Poses waypoints = new Poses(a);
        return curve(waypoints.point3, waypoints.parkCP,waypoints.point4).constant(waypoints.parkCP.heading());
    }

    /*public static Path path1(Alliance a) {
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
    //pose callback incase others were curious
    /*public static Path posecallbacktest(Alliance a, Command onReach) {
        Poses waypoints = new Poses(a);
        return withCallbacks(
                line(waypoints.start, waypoints.poseCallbackTestEnd).constant(waypoints.poseCallbackTestEnd),
                poseCallback(waypoints.poseCallbackTestTrigger, onReach, 2)
        );
    }*/
    /*public static Path posecallbacktest2(Alliance a, Command onReach) {
        Poses waypoints = new Poses(a);
        return Piece
    }*/
    /*
    public static Path posecallbacktest2(Alliance a, Command onReach) {
        Poses waypoints = new Poses(a);
        return withCallbacks(
                follower().pathBuilder().piecewiseCustom(waypoints.start)


                        .curveTo(waypoints.path1, waypoints.path1Control1, waypoints.path1Control2, waypoints.path1Control3)
                        .heading(Interpolator.piecewise()
                                .until(0.6096, Interpolator.tangent)
                                .until(1, Interpolator.constant(waypoints.path1End.heading())))


                        .lineTo(waypoints.point2)
                        .heading(Interpolator.piecewise()
                                .until(0.3865, Interpolator.linear(waypoints.point2Start.heading(),
                                        waypoints.point2Mid.heading()))
                                .until(1, Interpolator.constant(waypoints.point2.heading())))


                        .lineTo(waypoints.point3)
                        .constant(waypoints.point3.heading())

                        .done()
                        .toPath(),
                poseCallback(waypoints.poseCallbackTestTrigger, onReach, 2)
        );
    }
    /*
     */
    private static final class Poses {
        final Pose start;
        final Pose pollenEndingPose;
        final Pose path1Control1;
        final Pose path1Segment1Heading;
        final Pose path1Segment2Heading;
        final Pose endPointFlower;
        final Pose cpFlower;
        final Pose flowerTrigger;
        final Pose point2Segment1Start;
        final Pose point2Segment1End;
        final Pose point3;
        final Pose point3Control1;
        final Pose point4;

        final Pose parkCP;

        Poses(Alliance a) {
            PoseFactory poseFactory = poses(a);

            start = poseFactory.of(37.668, 7.794, 180);
            pollenEndingPose = poseFactory.of(8, 8.3998, -160); // "path1" in the visualizer
            path1Control1 = poseFactory.of(25.4753, 11.0007, 0);
            path1Segment1Heading = poseFactory.of(9.8632, 8.3998, 180);
            path1Segment2Heading = poseFactory.of(9.8632, 8.3998, 200);
            endPointFlower = poseFactory.of(34.5437, 104.0597, 78.9604);
            cpFlower = poseFactory.of(26.6689, 63.5582, 0);
            flowerTrigger = poseFactory.of(32.633, 94.595, 78);
            point2Segment1Start = poseFactory.of(34.5437, 104.0597, 180);
            point2Segment1End = poseFactory.of(34.5437, 104.0597, 90);
            point3 = poseFactory.of(47.1346, 119.6412, 90);
            point3Control1 = poseFactory.of(49.8421, 104.1041, 0);
            point4 = poseFactory.of(7.917, 122.961, 90);
            parkCP = poseFactory.of(48.119,104.606,90);



        }
    }
}
