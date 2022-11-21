package org.firstinspires.ftc.teamcode.kurio.pathfollow;

public class StopWayPoint extends HeadingControlledWayPoint {
    private final double stopThreshold;
    private final double angleThreshold;

    public StopWayPoint(double x, double y, double heading, double stopThreshold, double angleThreshold) {
        super(x, y, heading);

        this.stopThreshold = stopThreshold;
        this.angleThreshold = angleThreshold;
    }

    public double getStopThreshold() {
        return this.stopThreshold;
    }

    public double getAngleThreshold() {
        return this.angleThreshold;
    }
}
