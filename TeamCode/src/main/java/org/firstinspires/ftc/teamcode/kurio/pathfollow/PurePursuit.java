package org.firstinspires.ftc.teamcode.kurio.pathfollow;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.kurio.Robot;
import org.firstinspires.ftc.teamcode.kurio.math.MathUtil;
import org.firstinspires.ftc.teamcode.kurio.math.Point;
import org.firstinspires.ftc.teamcode.kurio.math.Pose;
import org.firstinspires.ftc.teamcode.kurio.util.ClassicalPID;

import java.util.List;

@Config
public class PurePursuit {
    private final Robot robot;

    private final double minDistance;
    private final double minAngleDifference;

    private final List<WayPoint> path;
    private final double followRadius;
    private int pathIndex; // which path segment is our follow point on? 0 is 0-1, 1 is 1-2, etc.

    private final StopWayPoint end;

    public static double xP = 0.082, xI = 0, xD = 0, yP = 0.069, yI = 0, yD = 0, hP = 0.68, hI = 0, hD = 0; // trust me

    public ClassicalPID xPID = new ClassicalPID(xP, xI, xD);
    public ClassicalPID yPID = new ClassicalPID(yP, yI, yD);
    public ClassicalPID headingPID = new ClassicalPID(hP, hI, hD);

    public PurePursuit(Robot robot, List<WayPoint> path, double followRadius) {
        if (path == null) throw new IllegalArgumentException();
        if (path.size() < 2) throw new IllegalArgumentException();
        if (followRadius <= 0) throw new IllegalArgumentException();
        if (!(path.get(path.size() - 1) instanceof StopWayPoint)) throw new IllegalArgumentException();

        this.robot = robot;
        this.path = path;
        this.followRadius = followRadius;

        this.end = (StopWayPoint) path.get(path.size() - 1);
        this.minDistance = end.getStopThreshold();
        this.minAngleDifference = end.getAngleThreshold();

        this.pathIndex = 0;

        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }

    public boolean update(Pose position) {
        if (atEnd(position)) {
            robot.getDrivetrainModule().setPowers(0, 0, 0);
            return false;
        }

        if (pathIndex == path.size() - 2) {
//            double xPow = xPID.calculateSpeed(end.getX() - position.getX());
//            double yPow = yPID.calculateSpeed(end.getY() - position.getY());
//            double anglePow = headingPID.calculateSpeed(end.getHeading()- position.getTheta());
//
//            robot.getDrivetrainModule().setPowers(xPow, yPow, anglePow);
            robot.getDrivetrainModule().setPowers(end.getX() - position.getX(), end.getY() - position.getY(), end.getHeading() - position.getTheta());

            return true;
        }

        Point target = targetPosition(position);
        Pose relDistancetoTarget = MathUtil.relativeDistance(position, target);

//        double xPow = xPID.calculateSpeed(relDistancetoTarget.getX());
//        double yPow = yPID.calculateSpeed(relDistancetoTarget.getY());
        double xPow = relDistancetoTarget.getX();
        double yPow = relDistancetoTarget.getY();

        double forwardAngle = MathUtil.angleWrap(relDistancetoTarget.getTheta());
        double backwardAngle = MathUtil.angleWrap(relDistancetoTarget.getTheta() + Math.PI);
        double autoAngle = Math.abs(forwardAngle) < Math.abs(backwardAngle) ? forwardAngle : backwardAngle;
        double desiredAngle = MathUtil.angleWrap(path.get(pathIndex + 1) instanceof HeadingControlledWayPoint
                || path.get(pathIndex + 1) instanceof StopWayPoint ?
                ((HeadingControlledWayPoint) path.get(pathIndex + 1)).getHeading() - position.getTheta() :
                autoAngle);

//        double anglePow = headingPID.calculateSpeed(desiredAngle);
        double anglePow = desiredAngle;

        robot.getDrivetrainModule().setPowers(-xPow, yPow, -anglePow); // we want to move in the opposite direction as our error

        return true;
    }

    private Point targetPosition(Point center) {
        if (pathIndex == path.size() - 2 || center.distance(path.get(path.size() - 1)) < followRadius)
            return path.get(path.size() - 1);

        // find the first path whose end is past the follow radius
        int lookAheadUntil = path.size() - 2;
        for (int i = pathIndex; i <= path.size() - 2; i++) {
            Point endSegment = path.get(i + 1);

            if (center.distance(endSegment) > followRadius) lookAheadUntil = i;
        }

        lookAheadUntil = Math.min(path.size() - 1, Math.max(lookAheadUntil, pathIndex + 1));

        // searches through all segments to find an intersection, starting from the end.
        for (int i = lookAheadUntil; i >= pathIndex; i--) {
            Point intersection = MathUtil.lineSegmentCircleIntersectionFar(path.get(i), path.get(i + 1), center, followRadius);

            if (intersection != null) return intersection;
        }

        // return end of path if there are no intersections
        return path.get(pathIndex + 1);
    }

    private boolean atEnd(Pose robot) {
        return robot.distance(end) <= minDistance && Math.abs(robot.getTheta() - end.getHeading()) <= minAngleDifference;
    }
}
