package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.FarPaths;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Configurable
public class Far extends CommandOpMode {
    Alliance a;
    Robot robot;
    boolean started = false;

    public Far(Alliance a) {this.a = a;}

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(FarPaths.startPose(a));
        robot.follower.update();
    }

    @Override
    public void initializeLoop() {
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.update();
    }

    @Override
    public void onStart() {
        started = true;
        schedule(
                infinite(robot::update),
                routine()
        );
    }

    @Override
    public void end() {
        if (robot == null) return;
        if (started) OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    /**
     * TODO: fill in once FarPaths has real paths, e.g.
     * {@code sequential(follow(robot.follower, FarPaths.toScore(a)), robot.shootCycle(), ...)}.
     */
    private Command routine() {
        return instant(() -> {});
    }
}
