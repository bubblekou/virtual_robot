package org.firstinspires.ftc.teamcode.kurio.math;

public class Twist {
    private final double xVel;
    private final double yVel;
    private final double thetaVel;

    public Twist(double xVel, double yVel, double thetaVel) {
        this.xVel = xVel;
        this.yVel = yVel;
        this.thetaVel = thetaVel;
    }

    public double getXVel() { return xVel; }

    public double getYVel() { return yVel; }

    public double getThetaVel() { return thetaVel; }

    public String toString(){
        return "xVel: " + xVel + " yVel: " + yVel + " thetaVel: " + thetaVel;
    }
}
