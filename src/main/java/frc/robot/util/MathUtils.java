package frc.robot.util;

public final class MathUtils {
    private MathUtils() {}

    /** Returns true if {@code a} and {@code b} differ by no more than {@code tol}. */
    public static boolean approxEqual(double a, double b, double tol) {
        return Math.abs(a - b) <= tol;
    }

    /** Returns true if {@code a} and {@code b} differ by no more than 1e-9. */
    public static boolean approxEqual(double a, double b) {
        return approxEqual(a, b, 1e-9);
    }
}
