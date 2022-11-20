package org.firstinspires.ftc.teamcode.kurio.opmode.motion;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.HeadingControlledWayPoint;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.PurePursuit;
import org.firstinspires.ftc.teamcode.kurio.pathfollow.StopWayPoint;

import java.util.Arrays;

@Autonomous(name = "SideWays", group = "KuriosityBot")
public class SideWays extends LinearOpMode {
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose(0, 0, 0), false);

        PurePursuit pp = new PurePursuit(robot, Arrays.asList(
                new HeadingControlledWayPoint(0, 0, 0),
                new HeadingControlledWayPoint(-20, 0, 0),
                new StopWayPoint(-50, 0, 0, 1.0, Math.toRadians(5))),
                8);

        waitForStart();

        while (pp.update(robot.getPose()) && !isStopRequested());
    }
}
