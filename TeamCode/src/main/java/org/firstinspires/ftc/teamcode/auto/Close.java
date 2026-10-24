package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.electrolightsftc.pulse.electropathing.AutoPathRuntime.follow;
import org.electrolightsftc.pulse.electropathing.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;

import org.electrolightsftc.pulse.electropathing.opmode.PulseOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.ClosePaths;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Configurable
public class Close extends CommandOpMode {
    Alliance a;
    Robot robot;
    boolean started = false;

    public Close(Alliance a) {this.a = a;

    }

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
                infinite(robot::update),
                routine()
        );
    }
    public Command shoot(){
        return robot.shootAll();
    }

    @Override
    public void end() {
        if (robot == null) return;
        if (started) OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    private Command routine() {
        return sequential(

                waitMs(500),
               shoot(),

                follow(robot.follower, ClosePaths.intakeLoading(a)),

                follow(robot.follower, ClosePaths.goNearFlower(a)),
                follow(robot.follower, ClosePaths.goInFlower(a)),
                follow(robot.follower, ClosePaths.moveback(a))
        );
    }
}
