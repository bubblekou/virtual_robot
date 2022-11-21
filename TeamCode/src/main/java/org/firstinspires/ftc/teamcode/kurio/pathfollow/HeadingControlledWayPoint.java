package org.firstinspires.ftc.teamcode.kurio.pathfollow;

import org.firstinspires.ftc.teamcode.kurio.math.Pose;

public class HeadingControlledWayPoint extends WayPoint {
    private final double heading;

    public HeadingControlledWayPoint(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public HeadingControlledWayPoint(HeadingControlledWayPoint HCW) {
        this(HCW.getX(), HCW.getY(), HCW.getHeading());
    }

    public HeadingControlledWayPoint(Pose pose) {
        this(pose.getX(), pose.getY(), pose.getTheta());
    }

    public double getHeading() {
        return this.heading;
    }
}
