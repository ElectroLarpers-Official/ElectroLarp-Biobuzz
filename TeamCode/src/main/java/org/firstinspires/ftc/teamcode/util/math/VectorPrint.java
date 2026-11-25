package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.math.Vector2D;

/** RevAmped {@code Vector2D.print}, for the RevAmped drive port ({@code subsystem/Drivetrain}). */
public final class VectorPrint {
    private VectorPrint() {}

    public static String print(Vector2D v) {
        return "x: " + v.x() + " y: " + v.y();
    }
}
