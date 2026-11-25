package org.firstinspires.ftc.teamcode.util.math;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * Pedro 2 {@code Vector.transform(Matrix)}, written with Pedro 3 types for the RevAmped drive port
 * ({@code subsystem/Drivetrain}). Reproduces Pedro 2 exactly, including its quirks; don't "fix" it here.
 */
public final class VectorTransform {
    private VectorTransform() {}

    /**
     * Pedro 2 multiplies the matrix by the column (x, y), then builds the result with its POLAR constructor
     * {@code new Vector(m0, m1)}: m0 is used as the magnitude and m1 as the angle, giving (m0 cos m1, m0 sin m1).
     * Pedro 3's {@code Vector2D.transform} returns (m0, m1) instead.
     * The product is summed like Pedro 2's {@code Matrix.multiply}: terms with a zero factor are skipped.
     */
    public static Vector2D transform(Vector2D v, Matrix matrix) {
        double[] col = {v.x(), v.y()};
        double[] multiply = new double[2];
        for (int i = 0; i < 2; i++) {
            double dpSum = 0;
            for (int k = 0; k < 2; k++) {
                double a = matrix.get(i, k);
                if (!(a == 0 || col[k] == 0)) {
                    dpSum += a * col[k];
                }
            }
            multiply[i] = dpSum;
        }
        return Vector2D.polar(multiply[0], multiply[1]);
    }
}
