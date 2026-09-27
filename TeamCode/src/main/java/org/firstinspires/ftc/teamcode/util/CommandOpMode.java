package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class CommandOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        reset();
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
            reset();
            end();
        }
    }

    public void reset() {
        Scheduler.reset();
    }

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
