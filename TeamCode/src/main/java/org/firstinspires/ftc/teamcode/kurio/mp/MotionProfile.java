package org.firstinspires.ftc.teamcode.kurio.mp;

public class MotionProfile {

    /**
     * Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
     *
     * @param maxAcceleration
     * @param maxVelocity
     * @param distance
     * @param currentDt
     * @return
     */
    public double profileDistance(double maxAcceleration,
                                  double maxVelocity,
                                  double distance,
                                  double currentDt) {
        // calculate the time it takes to accelerate to max velocity
        double accelerationDt = maxVelocity / maxAcceleration;
        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        // recalculate max velocity based on the time we have to accelerate and decelerate
        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // we decelerate at the same rate as we accelerate
        double deaccelerationDt = accelerationDt;

        // calculate the time that we're at max velocity
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;

        double deaccelerationTime = accelerationDt + cruiseDt;

        // check if we're still in the motion profile
        double entire_dt = accelerationDt + cruiseDt + deaccelerationDt;
        if (currentDt > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (currentDt < accelerationDt) {
            // use the kinematic equation for acceleration
            return 0.5 * maxAcceleration * Math.pow(currentDt, 2);
        }
        else if (currentDt < deaccelerationTime) { // if we're cruising
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            double cruiseCurrentDt = currentDt - accelerationDt;

            // use the kinematic equation for constant velocity
            return accelerationDistance + maxVelocity * cruiseCurrentDt;
        }
        else { // if we're decelerating
            double cruise_distance = maxVelocity * cruiseDt;
            double actualDeaccelerationTime = currentDt - deaccelerationTime;

            // use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruise_distance +
                    maxVelocity * actualDeaccelerationTime -
                    0.5 * maxAcceleration * Math.pow(actualDeaccelerationTime, 2);
        }
    }
}
