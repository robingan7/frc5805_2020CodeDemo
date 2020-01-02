package frc.lib.path;

import frc.lib.waypoint.*;
import frc.lib.motion_profile.*;
import frc.robot.Constants;

import java.util.Optional;

public class PathSegment {
    private Translation2D start;
    private Translation2D end;
    private Translation2D center;
    private Translation2D deltaStart;
    private Translation2D deltaEnd;
    private double maxSpeed;
    private boolean isLine;
    private MotionProfile speedController;
    private boolean extrapolateLookahead;
    private String marker;

    /**
     * Constructor for a linear segment
     *
     * @param x1       start x
     * @param y1       start y
     * @param x2       end x
     * @param y2       end y
     * @param maxSpeed maximum speed allowed on the segment
     */
    public PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionProfilePoint startState,
                       double endSpeed) {
        this.start = new Translation2D(x1, y1);
        this.end = new Translation2D(x2, y2);

        this.deltaStart = new Translation2D(start, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = true;
        createMotionProfiler(startState, endSpeed);
    }

    public PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionProfilePoint startState,
                       double endSpeed, String marker) {
        this.start = new Translation2D(x1, y1);
        this.end = new Translation2D(x2, y2);

        this.deltaStart = new Translation2D(start, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = true;
        this.marker = marker;
        createMotionProfiler(startState, endSpeed);
    }

    /**
     * Constructor for an arc segment
     *
     * @param x1       start x
     * @param y1       start y
     * @param x2       end x
     * @param y2       end y
     * @param cx       center x
     * @param cy       center y
     * @param maxSpeed maximum speed allowed on the segment
     */
    public PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
                       MotionProfilePoint startState, double endSpeed) {
        this.start = new Translation2D(x1, y1);
        this.end = new Translation2D(x2, y2);
        this.center = new Translation2D(cx, cy);

        this.deltaStart = new Translation2D(center, start);
        this.deltaEnd = new Translation2D(center, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = false;
        createMotionProfiler(startState, endSpeed);
    }

    public PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed,
                       MotionProfilePoint startState, double endSpeed, String marker) {
        this.start = new Translation2D(x1, y1);
        this.end = new Translation2D(x2, y2);
        this.center = new Translation2D(cx, cy);

        this.deltaStart = new Translation2D(center, start);
        this.deltaEnd = new Translation2D(center, end);

        this.maxSpeed = maxSpeed;
        extrapolateLookahead = false;
        isLine = false;
        this.marker = marker;
        createMotionProfiler(startState, endSpeed);
    }

    /**
     * @return max speed of the segment
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void createMotionProfiler(MotionProfilePoint start_state, double end_speed) {
        MaxVel_MaxAcc motionConstraints = new MaxVel_MaxAcc(maxSpeed,
                Constants.kPathFollowingMaxAccel);
        MotionProfileGoal goal_state = new MotionProfileGoal(getLength(), end_speed);
        speedController = MotionProfileGenerator.generateProfile(motionConstraints, goal_state, start_state);
        // System.out.println(speedController);
    }

    /**
     * @return starting point of the segment
     */
    public Translation2D getStart() {
        return start;
    }

    /**
     * @return end point of the segment
     */
    public Translation2D getEnd() {
        return end;
    }

    /**
     * @return the total length of the segment
     */
    public double getLength() {
        if (isLine) {
            return deltaStart.norm();
        } else {
            return deltaStart.norm() * Translation2D.getAngle(deltaStart, deltaEnd).getRadians();
        }
    }

    /**
     * Set whether or not to extrapolate the lookahead point. Should only be true for the last segment in the path
     *
     * @param val
     */
    public void extrapolateLookahead(boolean val) {
        extrapolateLookahead = val;
    }

    /**
     * Gets the point on the segment closest to the robot
     *
     * @param position the current position of the robot
     * @return the point on the segment closest to the robot
     */
    public Translation2D getClosestPoint(Translation2D position) {
        if (isLine) {
            Translation2D delta = new Translation2D(start, end);
            double u = ((position.x() - start.x()) * delta.x() + (position.y() - start.y()) * delta.y())
                    / (delta.x() * delta.x() + delta.y() * delta.y());
            if (u >= 0 && u <= 1)
                return new Translation2D(start.x() + u * delta.x(), start.y() + u * delta.y());
            return (u < 0) ? start : end;
        } else {
            Translation2D deltaPosition = new Translation2D(center, position);
            deltaPosition = deltaPosition.scale(deltaStart.norm() / deltaPosition.norm());
            if (Translation2D.cross(deltaPosition, deltaStart) * Translation2D.cross(deltaPosition, deltaEnd) < 0) {
                return center.translateBy(deltaPosition);
            } else {
                Translation2D startDist = new Translation2D(position, start);
                Translation2D endDist = new Translation2D(position, end);
                return (endDist.norm() < startDist.norm()) ? end : start;
            }
        }
    }

    /**
     * Calculates the point on the segment <code>dist</code> distance from the starting point along the segment.
     *
     * @param dist distance from the starting point
     * @return point on the segment <code>dist</code> distance from the starting point
     */
    public Translation2D getPointByDistance(double dist) {
        double length = getLength();
        if (!extrapolateLookahead && dist > length) {
            dist = length;
        }
        if (isLine) {
            return start.translateBy(deltaStart.scale(dist / length));
        } else {
            double deltaAngle = Translation2D.getAngle(deltaStart, deltaEnd).getRadians()
                    * ((Translation2D.cross(deltaStart, deltaEnd) >= 0) ? 1 : -1);
            deltaAngle *= dist / length;
            Translation2D t = deltaStart.rotateBy(Rotation2D.fromRadians(deltaAngle));
            return center.translateBy(t);
        }
    }

    /**
     * Gets the remaining distance left on the segment from point <code>point</code>
     *
     * @param point result of <code>getClosestPoint()</code>
     * @return distance remaining
     */
    public double getRemainingDistance(Translation2D position) {
        if (isLine) {
            return new Translation2D(end, position).norm();
        } else {
            Translation2D deltaPosition = new Translation2D(center, position);
            double angle = Translation2D.getAngle(deltaEnd, deltaPosition).getRadians();
            double totalAngle = Translation2D.getAngle(deltaStart, deltaEnd).getRadians();
            return angle / totalAngle * getLength();
        }
    }

    private double getDistanceTravelled(Translation2D robotPosition) {
        Translation2D pathPosition = getClosestPoint(robotPosition);
        double remainingDist = getRemainingDistance(pathPosition);
        return getLength() - remainingDist;

    }

    public double getSpeedByDistance(double dist) {
        if (dist < speedController.startPos()) {
            dist = speedController.startPos();
        } else if (dist > speedController.endPos()) {
            dist = speedController.endPos();
        }
        Optional<MotionProfilePoint> state = speedController.firstStateByPos(dist);
        if (state.isPresent()) {
            return state.get().getVelocity();
        } else {
            System.out.println("Velocity does not exist at that position!");
            return 0.0;
        }
    }

    public double getSpeedByClosestPoint(Translation2D robotPosition) {
        return getSpeedByDistance(getDistanceTravelled(robotPosition));
    }

    public MotionProfilePoint getEndState() {
        return speedController.endState();
    }

    public MotionProfilePoint getStartState() {
        return speedController.startState();
    }

    public String getMarker() {
        return marker;
    }

    public String toString() {
        if (isLine) {
            return "(" + "start: " + start + ", end: " + end + ", speed: " + maxSpeed // + ", profile: " +
                    // speedController
                    + ")";
        } else {
            return "(" + "start: " + start + ", end: " + end + ", center: " + center + ", speed: " + maxSpeed
                    + ")"; // + ", profile: " + speedController + ")";
        }
    }
}