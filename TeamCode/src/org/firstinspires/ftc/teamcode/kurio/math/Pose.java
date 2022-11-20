package org.firstinspires.ftc.teamcode.kurio.math;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Pose extends Point{
    private final double theta;

    private static final float FULL_FIELD = 144.0f;

    public Pose(double x, double y, double theta) {
        super(x, y);
        this.theta = theta;
    }

    public double getTheta() {
        return this.theta;
    }

    public Pose toFTCSystem() {
        double x = -this.getY() + (FULL_FIELD / 2.);
        double y = this.getX() - (FULL_FIELD / 2.);
        double heading = MathUtil.angleWrap(Math.PI - getTheta());
        return new Pose(x, y, heading);
    }

    public Vector2d getHeadingVector() {
        return new Vector2d(cos(theta), sin(theta));
    }

    public String toString(){
        return super.toString() + " theta: " + theta;
    }
}
