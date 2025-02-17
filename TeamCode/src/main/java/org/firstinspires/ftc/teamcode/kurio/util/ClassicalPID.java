package org.firstinspires.ftc.teamcode.kurio.util;

public class ClassicalPID {
    public final double P_FACTOR;
    public final double I_FACTOR;
    public final double D_FACTOR;

    private boolean reset;

    private double lastError;
    private double errorSum;
    private double errorChange;

    public ClassicalPID(double p, double i, double d) {
        P_FACTOR = p;
        I_FACTOR = i;
        D_FACTOR = d;

        this.reset = true;
    }

    public double calculateSpeed(double error) {
        errorSum += error;

        double p = error * P_FACTOR;
        double i = 0;
        double d = 0;

        if (!reset) {
            //update d to correct for overshoot
            d = D_FACTOR * (error - lastError);
        } else {
            reset = false;
            errorSum = error;
            d = 0;
        }

        //update i accordingly
        i = errorSum * I_FACTOR;

        lastError = error;
        errorChange = d;

        return p + i + d;
    }

    /**
     * Reset the PID controller using given default scale
     */

    public void reset() {
        reset = true;
        errorSum = 0;
    }

    public double getD() { return errorChange; }
}