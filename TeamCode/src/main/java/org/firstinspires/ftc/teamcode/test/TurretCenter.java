package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.CommandOpMode;

/**
 * Holds the turret servos at 0.5 (straight ahead), from init on. Use it to mesh the turret gears with the turret
 * pointing straight forward, or to check that 0.5 really is forward. If it's a little off after meshing, trim
 * TurretCalc.servoOffset in Panels rather than moving the center.
 */
@TeleOp(group = "Turret Test")
public class TurretCenter extends CommandOpMode {
    Turret turret;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);   // the constructor already sends 0.5
        turret.setPosition(Turret.CENTER);
    }

    @Override
    public void initializeLoop() {
        hold();
    }

    @Override
    public void execute() {
        hold();
    }

    private void hold() {
        turret.setPosition(Turret.CENTER);
        if (turret.isConnected()) {
            telemetry.addData("Turret servo", "\"%s\" at %.2f (straight ahead)", HardwareNames.TURRET, Turret.CENTER);
            telemetry.addLine("Mesh the gears with the turret pointing straight forward.");
        } else {
            telemetry.addLine("Turret servo \"" + HardwareNames.TURRET + "\" is not in the active robot config.");
        }
        telemetry.update();
    }
}
