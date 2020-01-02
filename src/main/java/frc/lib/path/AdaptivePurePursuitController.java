package frc.lib.path;

import frc.lib.waypoint.*;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
 * <p>
 * Basically, we find a spot on the path we'd like to follow and calculate the arc necessary to make us land on that
 * spot. The target spot is a specified distance ahead of us, and we look further ahead the greater our tracking error.
 * We also return the maximum speed we'd like to be going when we reach the target spot.
 */

public class AdaptivePurePursuitController {
    private static final double kReallyBigNumber = 1E6;

    public static class Command {
        public Twist2D delta = Twist2D.identity();
        public double cross_track_error;
        public double max_velocity;
        public double end_velocity;
        public Translation2D lookahead_point;
        public double remaining_path_length;

        public Command() {}

        public Command(Twist2D delta, double cross_track_error, double max_velocity, double end_velocity,
                       Translation2D lookahead_point, double remaining_path_length) {
            this.delta = delta;
            this.cross_track_error = cross_track_error;
            this.max_velocity = max_velocity;
            this.end_velocity = end_velocity;
            this.lookahead_point = lookahead_point;
            this.remaining_path_length = remaining_path_length;
        }
    }

    Path mPath;
    boolean mAtEndOfPath = false;
    final boolean mReversed;
    final Lookahead mLookahead;

    public AdaptivePurePursuitController(Path path, boolean reversed, Lookahead lookahead) {
        mPath = path;
        mReversed = reversed;
        mLookahead = lookahead;
    }

    /**
     * Gives the Pose2D.Delta that the robot should take to follow the path
     *
     * @param pose robot pose
     * @return movement command for the robot to follow
     */
    public Command update(Pose2D pose) {
        if (mReversed) {
            pose = new Pose2D(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation2D.fromRadians(Math.PI)));
        }

        final Path.TargetPointReport report = mPath.getTargetPoint(pose.getTranslation(), mLookahead);
        if (isFinished()) {
            // Stop.
            return new Command(Twist2D.identity(), report.closest_point_distance, report.max_speed, 0.0,
                    report.lookahead_point, report.remaining_path_distance);
        }

        final Arc arc = new Arc(pose, report.lookahead_point);
        double scale_factor = 1.0;
        // Ensure we don't overshoot the end of the path (once the lookahead speed drops to zero).
        if (report.lookahead_point_speed < 1E-6 && report.remaining_path_distance < arc.length) {
            scale_factor = Math.max(0.0, report.remaining_path_distance / arc.length);
            mAtEndOfPath = true;
        } else {
            mAtEndOfPath = false;
        }
        if (mReversed) {
            scale_factor *= -1;
        }

        return new Command(
                new Twist2D(scale_factor * arc.length, 0.0,
                        arc.length * getDirection(pose, report.lookahead_point) * Math.abs(scale_factor) / arc.radius),
                report.closest_point_distance, report.max_speed,
                report.lookahead_point_speed * Math.signum(scale_factor), report.lookahead_point,
                report.remaining_path_distance);
    }

    public boolean hasPassedMarker(String marker) {
        return mPath.hasPassedMarker(marker);
    }

    public static class Arc {
        public Translation2D center;
        public double radius;
        public double length;

        public Arc(Pose2D pose, Translation2D point) {
            center = getCenter(pose, point);
            radius = new Translation2D(center, point).norm();
            length = getLength(pose, point, center, radius);
        }
    }

    /**
     * Gives the center of the circle joining the lookahead point and robot pose
     *
     * @param pose  robot pose
     * @param point lookahead point
     * @return center of the circle joining the lookahead point and robot pose
     */
    public static Translation2D getCenter(Pose2D pose, Translation2D point) {
        final Translation2D poseToPointHalfway = pose.getTranslation().interpolate(point, 0.5);
        final Rotation2D normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction().normal();
        final Pose2D perpendicularBisector = new Pose2D(poseToPointHalfway, normal);
        final Pose2D normalFromPose = new Pose2D(pose.getTranslation(),
                pose.getRotation().normal());
        if (normalFromPose.isColinear(perpendicularBisector.normal())) {
            // Special case: center is poseToPointHalfway.
            return poseToPointHalfway;
        }
        return normalFromPose.intersection(perpendicularBisector);
    }

    /**
     * Gives the radius of the circle joining the lookahead point and robot pose
     *
     * @param pose  robot pose
     * @param point lookahead point
     * @return radius of the circle joining the lookahead point and robot pose
     */
    public static double getRadius(Pose2D pose, Translation2D point) {
        Translation2D center = getCenter(pose, point);
        return new Translation2D(center, point).norm();
    }

    /**
     * Gives the length of the arc joining the lookahead point and robot pose (assuming forward motion).
     *
     * @param pose  robot pose
     * @param point lookahead point
     * @return the length of the arc joining the lookahead point and robot pose
     */
    public static double getLength(Pose2D pose, Translation2D point) {
        final double radius = getRadius(pose, point);
        final Translation2D center = getCenter(pose, point);
        return getLength(pose, point, center, radius);
    }

    public static double getLength(Pose2D pose, Translation2D point, Translation2D center, double radius) {
        if (radius < kReallyBigNumber) {
            final Translation2D centerToPoint = new Translation2D(center, point);
            final Translation2D centerToPose = new Translation2D(center, pose.getTranslation());
            // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
            // check the sign of the cross-product between the normal vector and the vector from pose to point.
            final boolean behind = Math.signum(
                    Translation2D.cross(pose.getRotation().normal().toTranslation(),
                            new Translation2D(pose.getTranslation(), point))) > 0.0;
            final Rotation2D angle = Translation2D.getAngle(centerToPose, centerToPoint);
            return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
        } else {
            return new Translation2D(pose.getTranslation(), point).norm();
        }
    }

    /**
     * Gives the direction the robot should turn to stay on the path
     *
     * @param pose  robot pose
     * @param point lookahead point
     * @return the direction the robot should turn: -1 is left, +1 is right
     */
    public static int getDirection(Pose2D pose, Translation2D point) {
        Translation2D poseToPoint = new Translation2D(pose.getTranslation(), point);
        Translation2D robot = pose.getRotation().toTranslation();
        double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
        return (cross < 0) ? -1 : 1; // if robot < pose turn left
    }

    /**
     * @return has the robot reached the end of the path
     */
    public boolean isFinished() {
        return mAtEndOfPath;
    }
}