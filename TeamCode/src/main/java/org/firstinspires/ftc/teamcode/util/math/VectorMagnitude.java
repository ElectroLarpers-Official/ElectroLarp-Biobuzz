package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.math.Vector2D;

/**
 * Pedro 2 {@code Vector.getMagnitude()}, written with Pedro 3 types for the RevAmped drive port
 * ({@code subsystem/Drivetrain}). Reproduces Pedro 2 exactly; don't "fix" it here.
 */
public final class VectorMagnitude {
    private VectorMagnitude() {}

    /**
     * {@code Math.sqrt(x * x + y * y)}. Pedro 3's {@code magnitude()} uses {@code Math.hypot}, which can differ
     * in the last bit.
     */
    public static double magnitude(Vector2D v) {
        return Math.sqrt(v.x() * v.x() + v.y() * v.y());
    }
}
