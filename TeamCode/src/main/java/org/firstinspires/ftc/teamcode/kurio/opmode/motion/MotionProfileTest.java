package org.firstinspires.ftc.teamcode.kurio.opmode.motion;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.mp.MotionProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Autonomous(name = "MotionProfile", group = "KuriosityBot")
public class MotionProfileTest extends LinearOpMode {
    private static final Logger LOGGER = LoggerFactory.getLogger(MotionProfileTest.class);
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose(0, 0, 0), false);

        waitForStart();

        double maxAcceleration = 20;
        double maxVelocity = 20;
        double distance = 96;
        MotionProfile mp = new MotionProfile();
        long startTime = System.currentTimeMillis();
        double lastPosition = 0;
        while (!isStopRequested()) {
            double dt = (System.currentTimeMillis() - startTime) / 1000;
            double instantTargetPosition = mp.profileDistance(
                    maxAcceleration,
                    maxVelocity,
                    distance,
                    dt);

            if (Math.abs(instantTargetPosition - lastPosition) > 1) {
                LOGGER.info("Position: {}", instantTargetPosition);
                lastPosition = instantTargetPosition;
            }

            if (Math.abs(instantTargetPosition - distance) < 3) {
                // Close enough
                break;
            }

            Pose robotPose = robot.getPose();
            double xMotorPower = (instantTargetPosition - robotPose.getX()) * 0.2;
            robot.getDrivetrainModule().setPowers(xMotorPower, 0, 0);
        }
    }
}