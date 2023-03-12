package frc.robot.util.math;

import static java.lang.Math.toRadians;

public class Range {

    private double min;
    private double max;
    private boolean inclusive;

    public Range(double num1, double num2, boolean inclusive) {
        min = Math.min(num1, num2);
        max = Math.max(num1, num2);
        this.inclusive = inclusive;
    }

    public Range(double num1, double num2) {
        this(num1, num2, true);
    }

    public static Range fromDegrees(double num1, double num2) {
        return new Range(toRadians(num1), toRadians(num2));
    }

    public boolean isWithin(double num) {
        return inclusive ? num >= min && num <= max : num > min && num < max;
    }
}
