package org.firstinspires.ftc.teamcode.kurio.math;

public class Point {
    private final double x;
    private final double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distance(Point other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public String toString(){
        return "x: " + x + " y: " + y;
    }
}
