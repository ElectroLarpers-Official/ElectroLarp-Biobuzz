package org.firstinspires.ftc.teamcode.auto;

import static org.electrolightsftc.pulse.lumen.commands.Commands.infinite;
import static org.electrolightsftc.pulse.lumen.commands.Commands.instant;
import static org.electrolightsftc.pulse.lumen.commands.Commands.waitMs;
import static org.electrolightsftc.pulse.lumen.groups.Groups.sequential;
import static org.electrolightsftc.pulse.electropathing.PulseCommands.follow;
import org.electrolightsftc.pulse.electropathing.*;


import com.bylazar.configurables.annotations.Configurable;
import org.electrolightsftc.pulse.lumen.Command;
import org.electrolightsftc.pulse.lumen.Scheduler;

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
    private Command autoRoutine;

    public Close(Alliance a) {this.a = a;

    }

    @Override
    public void initialize() {
        Scheduler.reset();
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
        autoRoutine = routine();
        Scheduler.schedule(infinite(robot::update), autoRoutine);
    }

    @Override
    public void execute() {
        Scheduler.execute();
    }


    @Override
    public void end() {
        if (autoRoutine != null) Scheduler.cancel(autoRoutine);
        Scheduler.reset();
        if (robot == null) return;
        if (started) OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    private Command routine() {
        return sequential(
                waitMs(500),
                shootCycle(),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.intakeLoading(a), false),
                shootCycle(),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.goNearFlower(a, instant(() -> robot.flower.toggleFlower(true))), false),
                follow(robot.follower, ClosePaths.goInFlower(a), false),
                shootCycle(),
                sequential(robot.intake::off),
                follow(robot.follower, ClosePaths.moveback(a), false)
        );
    }

    private Command shootCycle() {
        return sequential(sequential(robot::startShooting), waitMs(Robot.SHOOT_DELAY_MS),
                sequential(robot::stopShooting));
    }

}
