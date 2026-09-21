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
import org.psilynx.psikit.core.Logger;

/** Autonomous entry point for the far-side starting position. */
@Configurable
public class Far extends CommandOpMode {
    Alliance a;
    Robot robot;
    boolean started = false; // false if the OpMode was stopped in init, so no pose worth handing to TeleOp

    public Far(Alliance a) {this.a = a;}

    @Override
    public void initialize() {
        Logger.recordMetadata("Alliance", a.name());
        Logger.recordMetadata("OpMode", "Far");
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
                infinite(robot::update), // must stay first: Scheduler runs commands in the order they were scheduled
                routine()
        );
    }

    @Override
    public void end() {
        if (robot == null) return;
        if (started) OpModeStorage.endPose = robot.follower.pose(); // TeleOp starts from here
        robot.stop();
    }

    /**
     * TODO: fill in once FarPaths has real paths, e.g.
     * {@code sequential(follow(robot.follower, FarPaths.toScore(a)), robot.shootOne(), ...)}.
     */
    private Command routine() {
        return instant(() -> {});
    }
}
