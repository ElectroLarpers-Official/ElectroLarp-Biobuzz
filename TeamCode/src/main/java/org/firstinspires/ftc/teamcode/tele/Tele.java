package org.firstinspires.ftc.teamcode.tele;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
// TODO: pick one shooter logic version (control.shotcheck or control.pasted) and delete the other; until then these imports must match Robot's.
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShooterCalc;
import org.firstinspires.ftc.teamcode.subsystem.control.shotcheck.ShotCheck;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

@Configurable
public class Tele extends CommandOpMode {
    /**
     * Rotates the field-centric frame so "stick forward" points away from the driver.
     * TODO: verify on the field which way each alliance's driver faces in Pedro coordinates.
     */
    public static double BLUE_FORWARD_OFFSET_DEG = 0;
    public static double RED_FORWARD_OFFSET_DEG = 180;

    /** Manual hood nudge per press. Reserved for driver 2 controls, which are not added yet. */
    public static double HOOD_STEP = 0.05;

    /** Rumble length while trying to shoot a shot that can't go in, ms. */
    public static int CANT_SHOOT_RUMBLE_MS = 250;

    Alliance a;
    Robot robot;
    GamepadEx gp1;
    Command shootOne;

    public Tele(Alliance a) {this.a = a;}

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, a);
        // TODO: fallback start pose (or a driver pose-reset button) for when no auto ran; aiming currently starts from (0, 0).
        if (OpModeStorage.endPose != null) {
            robot.follower.setPose(OpModeStorage.endPose);
            robot.follower.update();
        }
        gp1 = new GamepadEx(gamepad1);
        shootOne = robot.shootOne();
        gp1.left_trigger_button(FloatSupplier::isPress);
        gp1.right_trigger_button(FloatSupplier::isPress);
        schedule(infinite(() -> {
            robot.update();
            robot.drive(
                    -gp1.left_stick_y.getVal(),
                    -gp1.left_stick_x.getVal(),
                    -gp1.right_stick_x.getVal(),
                    Math.toRadians(a == Alliance.BLUE ? BLUE_FORWARD_OFFSET_DEG : RED_FORWARD_OFFSET_DEG));
            updateTelemetry();
        }));
    }

    @Override
    public void initializeLoop() {
        gp1.update();
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Pose source", OpModeStorage.endPose != null ? "auto / last run" : "NONE, starting at (0,0)");
        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void execute() {
        // TODO: add driver 2 controls (Shooter.partyMode(), HOOD_STEP hood nudge, ShooterCalc.verticalShiftStep) and manual
        //  fallbacks (turret/hood/flywheel) for when auto-aim or localization fails.
        gp1.update();

        if (gp1.left_trigger_button.isRisingEdge()) {
            robot.intake.intake();
        }
        if (gp1.left_trigger_button.isFallingEdge()) {
            robot.intake.stop();
        }
        if (gp1.right_trigger_button.isRisingEdge()) {
            robot.shootAll();
        }
        if (gp1.right_trigger_button.isFallingEdge()) {
            robot.stopShootAll();
        }
        if (gp1.left_bumper.isRisingEdge()) {
            robot.outtake();
        }
        if (gp1.left_bumper.isFallingEdge()) {
            robot.stopOuttake();
        }
        if (gp1.a.isRisingEdge() && !Scheduler.isScheduled(shootOne)) {
            schedule(shootOne);
        }
        boolean tryingToShoot = gp1.right_trigger_button.isTrue() || Scheduler.isScheduled(shootOne);
        if (tryingToShoot && !robot.shotAllowed() && !gamepad1.isRumbling()) {
            gamepad1.rumble(CANT_SHOOT_RUMBLE_MS);
        }

    }

    @Override
    public void end() {
        if (robot == null) return;
        OpModeStorage.endPose = robot.follower.pose();
        robot.stop();
    }

    private void updateTelemetry() {
        telemetry.addData("Alliance", a);
        telemetry.addData("Pose", robot.follower.pose());
        telemetry.addData("Shooter", "%.0f / %.0f ticks/s (%s)",
                robot.shooter.velocity(), robot.shooter.targetVelocity(),
                robot.shooter.atTarget() ? "at speed" : robot.shooter.isSpinning() ? "spinning up" : "off");
        telemetry.addData("Range now / projected / virtual", "%.1f / %.1f / %.1f",
                ShooterCalc.lastRange, ShooterCalc.lastProjectedRange, ShooterCalc.lastVirtualRange);
        telemetry.addData("Aim", !ShooterCalc.hiveMode ? "DECODE goal"
                : ShooterCalc.hiveAimFar() ? "HIVE far cell" : "HIVE audience cell");
        telemetry.addData("Shot", robot.readyToShoot() ? "READY" : ShotCheck.status());
        telemetry.addData("Tolerance turret deg / flywheel tps", "%.2f / %.0f",
                ShotCheck.turretToleranceDeg(), ShotCheck.flywheelToleranceTps());
        telemetry.addData("Turret heading angle", "%.1f", ShooterCalc.lastHeadingAngle);
        telemetry.addData("SOTM active", ShooterCalc.lastSotmActive);
        telemetry.addData("Flight time", "%.3f", ShooterCalc.lastFlightTime);
        telemetry.addData("Speed projected / aAlongV (filt)", "%.1f / %.1f",
                ShooterCalc.lastProjectedSpeed, ShooterCalc.lastAlongTrackAccel);
        telemetry.addData("RevAmped braking active", ShooterCalc.revAmpedActive);
        telemetry.addData("Hood comp burst deg / vel", "%.2f / %.3f",
                ShooterCalc.lastBurstHoodDeltaDeg, ShooterCalc.lastHoodCompTotal);
        telemetry.addData("Solver iters / converged", "%d / %b", ShooterCalc.lastIterations, ShooterCalc.lastConverged);
        telemetry.addData("Hood", "%.2f", robot.hood.position());
        telemetry.addData("Latch", robot.latch.isOpen() ? "open" : "closed");
        telemetry.addData("Intake", robot.intake.getState());
        telemetry.addData( "Transfer", robot.transfer.getState());
        telemetry.addData("Loop ms", robot.loopMs());
        telemetry.update();
    }
}
