package org.firstinspires.ftc.teamcode.kurio.math;


public class Line {
    private final double slope;
    private final double intercept;
    // If true, intercept is x-intercept. Otherwise (usually) it is y-intercept
    private final boolean isVerticalLine;

    public Line(Point p1, Point p2) {
        this(p1, (p2.getY() - p1.getY()) / (p2.getX() - p1.getX()));
    }

    public Line(Point point, double slope) {
        // We use negative infinity for the slope because it makes perpendicular slopes work
        if (Double.isInfinite(slope)) {
            this.slope = Double.NEGATIVE_INFINITY;
            this.intercept = point.getX();
            this.isVerticalLine = true;
        } else {
            this.slope = slope;
            this.intercept = point.getX() - point.getX() * slope;
            this.isVerticalLine = false;
        }
    }

    public Line(double slope, double yIntercept) {
        if (Double.isInfinite(slope)) {
            throw new IllegalArgumentException("Cannot use slope/y-intercept form for vertical line");
        }
        this.slope = slope;
        this.intercept = yIntercept;
        this.isVerticalLine = false;
    }

    public double perpendicularSlope() {
        return -1.0 / slope;
    }

    public double evaluate(double x) {
        if (isVerticalLine) {
            throw new IllegalArgumentException("Cannot evalute values of vertical line");
        } else {
            return x * slope + intercept;
        }
    }

    public Point intersect(Line l2) {
        // Vertical lines will trigger this too, as they always have slope NEGATIVE_INFINITY
        if (MathUtil.approxEquals(this.slope, l2.slope)) {
            if (MathUtil.approxEquals(this.intercept, l2.intercept)) {
                throw new IllegalArgumentException("Equal lines intersect everywhere");
            } else { // If parallel lines
                throw new IllegalArgumentException("Parallel lines do not intersect");
            }
        }

        if (isVerticalLine) {
            return new Point(intercept, l2.evaluate(intercept));
        } else if (l2.isVerticalLine) {
            return new Point(l2.intercept, this.evaluate(intercept));
        } else {
            double xIntersect = (l2.intercept - this.intercept) / (this.slope - l2.slope);
            return new Point(xIntersect, this.evaluate(xIntersect));
        }
    }

    public Point nearestLinePoint(Point p) {
        Line perpContainer = new Line(p, this.perpendicularSlope());
        return this.intersect(perpContainer);
    }

    public boolean isVertical() {
        return this.isVerticalLine;
    }

    public static Point midpoint(Point p1, Point p2) {
        return new Point((p1.getX() + p2.getX()) / 2, (p1.getY() + p2.getY()) / 2);
    }

    public static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Line line = (Line) o;
        return MathUtil.approxEquals(line.slope, slope) &&
                MathUtil.approxEquals(line.intercept, intercept);
    }
}
