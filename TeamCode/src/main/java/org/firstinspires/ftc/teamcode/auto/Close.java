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
    public static double RPM_1 = 2000;
    public static double RPM_2 = 2000;
    public static boolean autoRPM = false;
    Alliance a;
    Robot robot;
    boolean started = false;
    private boolean secondRpm = false;
    private Command autoRoutine;

    public Close(Alliance a) {this.a = a;

    }

    @Override
    public void initialize() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, a);
        robot.follower.setPose(ClosePaths.startPose(a));
        robot.follower.update();
        robot.flower(false);
    }

    @Override
    public void initializeLoop() {
        autoRPM = true;
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.update();
        robot.flower.update();
    }

    @Override
    public void onStart() {

        started = true;
        autoRoutine = routine();
        Scheduler.schedule(autoRoutine);
    }

    @Override
    public void execute() {
        onUpdate();
    }

    private void onUpdate() {
        robot.update();
        robot.shooter.setTargetRpm(secondRpm ? RPM_2 : RPM_1);
        robot.shooter.update();
    }


    @Override
    public void end() {
        autoRPM = false;
        if (autoRoutine != null) Scheduler.cancel(autoRoutine);
        Scheduler.reset();
        if (robot == null) return;
        if (started) OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    private Command routine() {
        return sequential(
                instant(()->robot.flower(false)),
                waitMs(3000),
                shootCycle(),
                waitMs(500),
                instant(()->robot.latch.close()),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.intakeLoading(a), false),
                waitMs(1000),
                instant(() -> secondRpm = true),
               //shootCycle(),
                instant(()->robot.latch.close()),
                sequential(robot.intake::on),
                instant(()->robot.flower.setDown(true)),
                follow(robot.follower, ClosePaths.goNearFlower(a, instant(() -> robot.flower.setDown(true))), false),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.goInFlower(a), false,0.7),
                shootCycle(),
                waitMs(2000),
                sequential(robot.intake::off),
                shootCycle(),
                instant(() -> robot.flower.setDown(false)),
                follow(robot.follower, ClosePaths.park(a), false)
        );
    }

    private Command shootCycle() {
        return sequential(sequential(robot::startShooting), waitMs(Robot.SHOOT_DELAY_MS),
                sequential(robot::stopShooting));
    }

}
