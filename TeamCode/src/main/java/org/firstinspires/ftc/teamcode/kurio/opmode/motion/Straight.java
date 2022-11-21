package org.firstinspires.ftc.teamcode.kurio.opmode.motion;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.PurePursuit;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.StopWayPoint;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.WayPoint;

import java.util.Arrays;

@Autonomous(name = "Straight", group = "KuriosityBot")
public class Straight extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose(0, 0, 0), false);

        PurePursuit pp = new PurePursuit(robot, Arrays.asList(new WayPoint(0, 0),
                new WayPoint(0, 20),
                new WayPoint(0, 35),
//                new WayPoint(-30, 60, 0)
                new StopWayPoint(0, 50, 0, 1.0, Math.toRadians(5.0))), 6);

        waitForStart();

        while (pp.update(robot.getPose()) && !isStopRequested());
    }
}
