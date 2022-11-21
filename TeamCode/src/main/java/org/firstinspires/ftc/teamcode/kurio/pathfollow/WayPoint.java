package org.firstinspires.ftc.teamcode.kurio.pathfollow;

import org.firstinspires.ftc.teamcode.kurio.math.Point;
public class WayPoint extends Point {
    public WayPoint(double x, double y) {
        super(x, y);
    }

    public WayPoint(WayPoint wayPoint) {
        this(wayPoint.getX(), wayPoint.getY());
    }

    public WayPoint(Point point) {
        this(point.getX(), point.getY());
    }

    public Point getPoint() {
        return new Point(getX(), getY());
    }
}
