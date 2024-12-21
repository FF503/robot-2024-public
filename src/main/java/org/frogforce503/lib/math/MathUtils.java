package org.frogforce503.lib.math;

import static java.lang.Math.*;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Miscellaneous math functions
 *
 * @since 0.1
 */
public class MathUtils {
    /**
     * Default epsilon for {@link #epsilonEquals(double, double)}
     *
     * @since 0.1
     */
    public static final double EPSILON = 1e-9;

    private MathUtils() {}

    public static double maxOf(double... values) {
        return Arrays.stream(values).max().orElse(Double.MIN_VALUE);
    }

    public static double minOf(double... values) {
        return Arrays.stream(values).min().orElse(Double.MAX_VALUE);
    }

    /**
     * Rounds a number to a specified number of digits after or before the decimal point.
     * 
     * @implSpec
     * <b>Summary:</b>
     * <ul>
     *     <li>If {@code digits > 0}, retains that many digits after the decimal point.</li>
     *     <li>If {@code digits == 0}, removes all digits after the decimal point.</li>
     *     <li>If {@code digits < 0}, rounds to remove digits before the decimal point.</li>
     * </ul>
     * 
     * <b>Example:</b>
     * <ul>
     *     <li>{@code round(1.323, 1)} → {@code 1.300}</li>
     *     <li>{@code round(1.323, 2)} → {@code 1.320}</li>
     *     <li>{@code round(1.323, 3)} → {@code 1.323}</li>
     *     <li>{@code round(1.323, 0)} → {@code 1.000}</li>
     *     <li>{@code round(1.323, -1)} → {@code 0.000}</li>
     * </ul>
     *
     * @param num Number to round
     * @param digits Number of digits to round by
     * @return Rounded number
     */
    public static double round(double num, double digits) {
        return Math.round(num * Math.pow(10, digits)) / Math.pow(10, digits);
    }

    /**
     * Clamps a value between a minimum and a maximum value.
     *
     * @param value The value to clamp.
     * @param min   The minimum value of the range. This value must be less than
     *              max.
     * @param max   The maximum value of the range. This value must be less than
     *              min.
     * @return the clamped value.
     * @since 0.2
     */
    public static double clamp(double value, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min must not be greater than max");
        }

        return max(min, min(value, max));
    }

    public static boolean equalsOneOf(double toCheckValue, double... options) {
        return Arrays.stream(options).anyMatch(v -> v == toCheckValue);
    }

    /**
     * More efficient hypot method without the square root, generally for the purpose of checking if points are within a distance
     * @param x
     * @param y
     * @return c^2
     */
    public static double hypotSquared(double x, double y) {
        return x * x + y * y;
    }

    public static double distanceSquared(Translation2d a, Translation2d b) {
        return hypotSquared(a.getX() - b.getX(), a.getY() - b.getY());
    }

    /**
     * Checks if two numbers are equal to each other using the default epsilon.
     *
     * @param a The first number.
     * @param b The second number.
     * @return If the two numbers are equal.
     * @since 0.1
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two numbers are equal to each other.
     *
     * @param a       The first number
     * @param b       The second number
     * @param epsilon The epsilon the comparison will use. Think of this as the
     *                allowable difference between the two numbers.
     * @return If the numbers are equal.
     * @since 0.1
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return abs(a - b) < epsilon;
    }

    /**
     * Returns if the value is in the range [lowerBound, upperBound].
     *
     * @param lowerBound The lower bound of the range.
     * @param upperBound The upper bound of the range.
     * @param value      The value.
     * @return If the value is in the range.
     * @since 0.1
     */
    public static boolean isInRange(double lowerBound, double upperBound, double value) {
        return lowerBound <= value && value <= upperBound;
    }

    /**
     * Solves the equation <code>0 = ax<sup>2</sup> + bx + c</code> for x and
     * returns the real results.
     *
     * @param a the a coefficient
     * @param b the b coefficient
     * @param c the c coefficient
     * @return the real roots of the equation
     */
    public static double[] quadratic(double a, double b, double c) {
        double sqrt = Math.sqrt(b * b - 4 * a * c);
        if (Double.isNaN(sqrt)) {
            // No roots
            return new double[0];
        }

        return new double[]{(-b + sqrt) / (2 * a), (-b - sqrt) / (2 * a)};
    }


    /**
     * Cross product with two Translation2d objects as vectors
     * @param v
     * @param w
     * @return
     */
    public static double cross(Translation2d v, Translation2d w) {
        return v.getX() * w.getY() - v.getY() * w.getX();
    }


    /**
     * 
     * See:
     * https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/lib/geometry/Pose2d.java#L82
     * and https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static Twist2d poseLog(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < 1E-9) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }

    public static double cardinalSnap(double angleDeg) {
        return 45 * (double) Math.round(angleDeg / 45.0);
    }

    public static Rotation2d cardinalSnap(Rotation2d angle) {
        return Rotation2d.fromDegrees(cardinalSnap(angle.getDegrees()));
    }

}
