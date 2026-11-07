package org.firstinspires.ftc.teamcode.auto;

import static org.electrolightsftc.pulse.lumen.commands.Commands.infinite;
import static org.electrolightsftc.pulse.lumen.commands.Commands.waitMs;
import static org.electrolightsftc.pulse.lumen.groups.Groups.sequential;
import static org.electrolightsftc.pulse.electropathing.PulseCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import org.electrolightsftc.pulse.lumen.Command;
import org.electrolightsftc.pulse.lumen.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.OneTipPaths;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Configurable
public class OneTip extends CommandOpMode {
    public static double FIXED_RPM = 4000;

    Alliance a;
    Robot robot;
    boolean started = false;
    private Command autoRoutine;

    public OneTip(Alliance a) {
        this.a = a;
    }

    @Override
    public void initialize() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(OneTipPaths.startPose(a));
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
        Scheduler.schedule(infinite(robot::update), infinite(this::holdRpm), autoRoutine);
    }

    private void holdRpm() {
        robot.shooter.setTargetRpm(FIXED_RPM);
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
                waitMs(2000),
                follow(robot.follower, OneTipPaths.path1(a), false),
                //shootCycle(),
                follow(robot.follower, OneTipPaths.path2(a), false)
        );
    }

    private Command shootCycle() {
        return sequential(sequential(robot::startShooting), waitMs(Robot.SHOOT_DELAY_MS),
                sequential(robot::stopShooting));
    }
}
