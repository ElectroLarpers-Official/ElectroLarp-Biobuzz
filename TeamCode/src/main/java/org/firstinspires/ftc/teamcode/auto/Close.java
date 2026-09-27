package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.ClosePaths;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

/** Autonomous entry point for the close-side starting position. */
@Configurable
public class Close extends CommandOpMode {
    Alliance a;
    Robot robot;
    boolean started = false; // false if the OpMode was stopped in init, so no pose worth handing to TeleOp

    public Close(Alliance a) {this.a = a;}

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(ClosePaths.startPose(a));
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
     * TODO: fill in once ClosePaths has real paths, e.g.
     * {@code sequential(follow(robot.follower, ClosePaths.toScore(a)), robot.shootOne(), ...)}.
     */
    private Command routine() {
        return instant(() -> {});
    }
}
