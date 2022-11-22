package org.firstinspires.ftc.teamcode.kurio.math;

public class MathUtil {
    public static final double EPSILON = 1e-6;
    public static final double HIGH_EPSILON = 1e-2;

    public static double angleWrap(double angle) {
        double tmp = angle % (Math.PI * 2);

        if (Math.abs(tmp) > Math.PI) tmp -= Math.copySign(Math.PI * 2, tmp);

        return tmp;
    }
    public static double clip(double d) {
        return Math.min(Math.max(d, -1), 1);
    }

    public static double clampAbove(double d, double threshold) {
        return Math.abs(d) < threshold ? Math.copySign(threshold, d) : d;
    }

    public static double clampBelow(double d, double threshold) {
        return Math.abs(d) < threshold ? 0 : d;
    }

    public static double powKeepSign(double d, double power) {
        // In case d is super small, just make it zero
        if (Math.abs(d) < 1e-12) {
            return 0;
        }
        return Math.copySign(Math.pow(Math.abs(d), power), d);
    }

    public static double zeroify(double d, double thresh) {
        return (Math.abs(d) < thresh) ? 0 : d;
    }

    public static boolean approxEquals(double d1, double d2) {
        if (Double.isInfinite(d1)) {
            // Infinity - infinity is NaN, so we need a special case
            return d1 == d2;
        } else {
            return Math.abs(d1 - d2) < EPSILON;
        }
    }

    public static Point lineSegmentCircleIntersectionFar(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.getX() - pointA.getX();
        double baY = pointB.getY() - pointA.getY();
        double caX = center.getX() - pointA.getX();
        double caY = center.getY() - pointA.getY();

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        // If there are any cases where disc is very small but negative(ex. -1.17e16), we need to set it to zero
        if (disc < 0 && Math.abs(disc) > EPSILON) return null;
        else if (Math.abs(disc) < EPSILON) disc = 0;
        // if disc == 0 ... dealt with later
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        Point p1 = new Point(pointA.getX() - baX * abScalingFactor1, pointA.getY() - baY * abScalingFactor1);
        if (disc == 0) return p1;
        Point p2 = new Point(pointA.getX() - baX * abScalingFactor2, pointA.getY() - baY * abScalingFactor2);
        return p1.distance(pointB) < p2.distance(pointB) ? p1 : p2;
    }

    public static Point lineSegmentCircleIntersectionNear(Point pointA, Point pointB, Point center, double radius) {
        double baX = pointB.getX() - pointA.getX();
        double baY = pointB.getY() - pointA.getY();
        double caX = center.getX() - pointA.getX();
        double caY = center.getY() - pointA.getY();

        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;

        double pBy2 = bBy2 / a;
        double q = c / a;

        double disc = pBy2 * pBy2 - q;
        // If there are any cases where disc is very small but negative(ex. -1.17e16), we need to set it to zero
        if (disc < 0 && Math.abs(disc) > EPSILON) return null;
        else if (Math.abs(disc) < EPSILON) disc = 0;
        // if disc == 0 ... dealt with later
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;

        Point p1 = new Point(pointA.getX() - baX * abScalingFactor1, pointA.getY() - baY * abScalingFactor1);
        if (disc == 0) return p1;
        Point p2 = new Point(pointA.getX() - baX * abScalingFactor2, pointA.getY() - baY * abScalingFactor2);
        return p1.distance(pointA) < p2.distance(pointA) ? p1 : p2;
    }

    private static int sign(double n) {
        return n < 0 ? -1 : 1;
    }

    public static boolean between(double r1, double r2, double val, double threshold) {
        return val > (Math.min(r1, r2) - threshold) && val < (Math.max(r1, r2) + threshold);
    }

    public static Pose relativeDistance(Pose pose, Point target) {
        double distance = target.distance(pose);
        double angle = Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX());

        double x = distance * Math.cos(angle);
        double y = distance * Math.sin(angle);

        return new Pose(x, y, angle);
    }

    public static double max(double... values) {
        double max = 1;
        for (double val : values) max = Math.max(max, val);

        return max;
    }
}

