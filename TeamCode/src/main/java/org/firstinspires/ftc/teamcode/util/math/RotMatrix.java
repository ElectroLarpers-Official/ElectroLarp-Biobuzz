package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.math.Matrix;

/**
 * RevAmped {@code MathUtil.rotMatrix}, copied as-is for the RevAmped drive port ({@code subsystem/Drivetrain}).
 * Reproduces the original exactly, including its quirk; don't "fix" it here.
 */
public final class RotMatrix {
    private RotMatrix() {}

    /**
     * Note the bottom-left entry is {@code -sin}, so this is not a true rotation matrix (Pedro 3's
     * {@code Matrix.rotation} is, and would give different numbers).
     */
    public static Matrix rotMatrix(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double[][] vals = new double[][] {
                {cos, -sin},
                {-sin, cos}
        };
        return new Matrix(vals);
    }
}
