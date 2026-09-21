package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class CommandOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        reset(); // Scheduler is static, so clear anything left over from the last OpMode
        try {
            preInit();
            initialize();

            while (opModeInInit()) {
                initializeLoop();
            }

            waitForStart();

            if (opModeIsActive()) {
                onStart();

                while (opModeIsActive()) {
                    execute();
                    Scheduler.execute();
                }
            }
        } finally {
            // Runs on every exit (stop pressed in init, stop pressed while running, or an exception).
            reset();
            end();
        }
    }

    /**
     * Cancels all previous commands
     */
    public void reset() {
        Scheduler.reset();
    }

    /**
     * Schedules commands to the scheduler
     */
    public void schedule(Command... commands) {
        Scheduler.schedule(commands);
    }

    public abstract void initialize();

    public void initializeLoop() {}

    public void execute() {}

    public void onStart() {}

    public void end() {}

    public void preInit() {}
}
