package org.firstinspires.ftc.teamcode.auto;

import static org.electrolightsftc.pulse.lumen.commands.Commands.infinite;
import static org.electrolightsftc.pulse.lumen.commands.Commands.instant;
import static org.electrolightsftc.pulse.lumen.commands.Commands.waitMs;
import static org.electrolightsftc.pulse.lumen.commands.Commands.waitUntil;
import static org.electrolightsftc.pulse.lumen.groups.Groups.sequential;
import static org.electrolightsftc.pulse.electropathing.PulseCommands.follow;
import org.electrolightsftc.pulse.electropathing.*;


import com.bylazar.configurables.annotations.Configurable;
import org.electrolightsftc.pulse.lumen.Command;
import org.electrolightsftc.pulse.lumen.Scheduler;

import org.electrolightsftc.pulse.electropathing.opmode.PulseOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystem.aim.turret.TurretCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Configurable
public class Close extends CommandOpMode {
    public static double RPM_1 = 4680;
    public static double RPM_2 = 4250;
    public static double RPM_3 = 4410;
    public static double FEED_DURATION_MS = 400;
    public static boolean autoRPM = false;

    private static final double INITIAL_TURRET_OFFSET_DEG = 5;
    private static final double POST_INTAKE_TURRET_ANGLE_DEG = -169; // Clockwise from forward.
    private static final double LATCH_OPEN_DELAY_MS = 320;
    Alliance a;
    Robot robot;
    boolean started = false;
    private enum ShooterStage { OPENING, FLOWER, FINAL }
    private ShooterStage shooterStage = ShooterStage.OPENING;
    private Command autoRoutine;

    public Close(Alliance a) {this.a = a;

    }

    @Override
    public void initialize() {
        Scheduler.reset();
        robot = new Robot(hardwareMap, a);
        robot.setTurretOffsetOverride(INITIAL_TURRET_OFFSET_DEG);
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
        shooterStage = ShooterStage.OPENING;
        started = true;
        autoRoutine = routine();
        Scheduler.schedule(autoRoutine);
    }

    @Override
    public void execute() {
        onUpdate();
        Scheduler.execute();
        telemetry.addData("Pose x", robot.follower.pose().x());
        telemetry.addData("Pose y", robot.follower.pose().y());
        telemetry.addData("Heading deg", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Following", robot.follower.following());
        telemetry.addData("Busy", robot.follower.isBusy());
        telemetry.addData("Vel x", robot.follower.velocity().vx);
        telemetry.addData("Vel y", robot.follower.velocity().vy);
        telemetry.addData("Shooter RPM", "%.0f actual / %.0f target", robot.shooter.rpm(), robot.shooter.targetRpm());
        Double fixedAngle = robot.fixedTurretAngleDeg();
        telemetry.addData("Turret target", fixedAngle == null ? "Tracking (pre-intake)"
                : String.format("Fixed %.1f deg", fixedAngle));
        telemetry.addData("Turret command", "%.1f deg%s", TurretCalc.commandDeg(),
                TurretCalc.outOfRange() ? " (at travel limit)" : "");
        telemetry.addData("Turret servos", robot.turret.servoCount());
        telemetry.update();
    }

    private void onUpdate() {
        robot.update();
        robot.shooter.setTargetRpm(targetRpm());
        robot.shooter.update();
    }

    private double targetRpm() {
        switch (shooterStage) {
            case FLOWER: return RPM_2;
            case FINAL: return RPM_3;
            case OPENING:
            default: return RPM_1;
        }
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
                waitMs(4000),
                shootCycle(),
                waitMs(500),
                instant(()->robot.latch.close()),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.intakeLoading(a), false,0.6),
                instant(() -> robot.setFixedTurretAngle(POST_INTAKE_TURRET_ANGLE_DEG)),
                waitMs(1000),
                instant(() -> shooterStage = ShooterStage.FLOWER),
               //shootCycle(),
                instant(()->robot.latch.close()),
                sequential(robot.intake::on),
                instant(()->robot.flower.setDown(true)),
                follow(robot.follower, ClosePaths.goNearFlower(a, instant(() -> robot.flower.setDown(true))), false),
                //shootCycle(),
                sequential(robot.intake::on),
                follow(robot.follower, ClosePaths.approachFlower(a), true, 0.7),
                shootCycle2(),
                instant(() -> shooterStage = ShooterStage.FINAL),
                follow(robot.follower, ClosePaths.finishFlower(a), false, 0.7),

                sequential(robot.intake::on),
                waitMs(2000),
                sequential(robot.intake::on),
                shootCycle(),

                follow(robot.follower, ClosePaths.park(a), false),
                instant(() -> robot.flower.setDown(false)),
                sequential(robot.intake::off)
        );
    }

    private Command shootCycle() {
        return sequential(
                instant(() -> {
                    robot.intake.off();
                    robot.latch.open();
                }),
                waitMs(LATCH_OPEN_DELAY_MS),
                instant(robot::startShooting),
                waitMs(FEED_DURATION_MS),
                instant(robot::stopShooting)
        );
    }

    private Command shootCycle2() {
        return sequential(waitMs(200), shootCycle());
    }

}
