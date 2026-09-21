package org.firstinspires.ftc.teamcode.tele;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.control.ShooterCalc;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.input.FloatSupplier;
import org.firstinspires.ftc.teamcode.util.input.GamepadEx;

/**
 * Driver 1 control:
 * <ul>
 *   <li>Left stick: translate (field-centric). Right stick X: turn.</li>
 *   <li>Left trigger (hold): run intake.</li>
 *   <li>Right trigger (hold): open latch, run transfer + intake, i.e. feed the shooter.</li>
 *   <li>A: shoot exactly one ball.</li>
 *   <li>Left bumper (hold): reverse intake + transfer to clear a jam.</li>
 * </ul>
 * The turret, hood and flywheel have no manual controls: {@link Robot#update()} aims them from the pose, and the
 * flywheel spins for as long as the OpMode is running (not during init).
 */
@Configurable
public class Tele extends CommandOpMode {
    /**
     * Rotates the field-centric frame so "stick forward" points away from the driver.
     * TODO: verify on the field which way each alliance's driver faces in Pedro coordinates.
     */
    public static double BLUE_FORWARD_OFFSET_DEG = 0;
    public static double RED_FORWARD_OFFSET_DEG = 180;

    public static double HOOD_STEP = 0.05;

    Alliance a;
    Robot robot;
    GamepadEx gp1;
    Command shootOne;

    public Tele(Alliance a) {this.a = a;}

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, a);
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
        //updates
        gp1.update();

        //====================GAMEPAD_1===================
        //Intake (hold)
        if (gp1.left_trigger_button.isRisingEdge()) {
            robot.intake.intake();
        }
        if (gp1.left_trigger_button.isFallingEdge()) {
            robot.intake.stop();
        }
        //Feed the shooter: latch open + transfer + intake (hold)
        if (gp1.right_trigger_button.isRisingEdge()) {
            robot.feed();
        }
        if (gp1.right_trigger_button.isFallingEdge()) {
            robot.stopFeed();
        }
        //Outtake, clears a jam (hold)
        if (gp1.left_bumper.isRisingEdge()) {
            robot.outtake();
        }
        if (gp1.left_bumper.isFallingEdge()) {
            robot.stopOuttake();
        }
        //Shoot one (a press while a shot is still running is ignored)
        if (gp1.a.isRisingEdge() && !Scheduler.isScheduled(shootOne)) {
            schedule(shootOne);
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
        telemetry.addData("Hive", robot.hiveVision.getState());
        telemetry.addData("Shooter", "%.0f / %.0f ticks/s (%s)",
                robot.shooter.velocity(), robot.shooter.targetVelocity(),
                robot.shooter.atTarget() ? "at speed" : robot.shooter.isSpinning() ? "spinning up" : "off");
        telemetry.addData("Range now / projected / virtual", "%.1f / %.1f / %.1f",
                ShooterCalc.lastRange, ShooterCalc.lastProjectedRange, ShooterCalc.lastVirtualRange);
        telemetry.addData("Turret heading angle", "%.1f", ShooterCalc.lastHeadingAngle);
        telemetry.addData("SOTM active", ShooterCalc.lastSotmActive);
        telemetry.addData("Solver iters / converged", "%d / %b", ShooterCalc.lastIterations, ShooterCalc.lastConverged);
        telemetry.addData("Hood", "%.2f", robot.hood.position());
        telemetry.addData("Latch", robot.latch.isOpen() ? "open" : "closed");
        telemetry.addData("Intake", robot.intake.getState());
        telemetry.addData( "Transfer", robot.transfer.getState());
        telemetry.addData("Loop ms", robot.loopMs());
        telemetry.update();
    }
}
