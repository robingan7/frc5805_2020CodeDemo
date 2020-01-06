package frc.robot.statesAndMachanics;

import frc.lib.waypoint.Twist2D;
import frc.lib.waypoint.Pose2D;
import frc.lib.waypoint.Rotation2D;
import frc.lib.utility.DriveSignal;
import frc.robot.Constants;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */

public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2D forwardKinematics(double left_wheel_delta, double right_wheel_delta) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (Constants.kDriveWheelTrackWidthInches * Constants.kTrackScrubFactor);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    public static Twist2D forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2D(dx, 0.0, delta_rotation_rads);
    }

    public static Twist2D forwardKinematics(Rotation2D prev_heading, double left_wheel_delta, double right_wheel_delta,
                                            Rotation2D current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist2D(dx, dy, prev_heading.inverse().rotateBy(current_heading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2D and previous rotation.
     */
    public static Pose2D integrateForwardKinematics(Pose2D current_pose,
                                                    Twist2D forward_kinematics) {
        return current_pose.transformBy(Pose2D.exp(forward_kinematics));
    }

    /**
     * Uses inverse kinematics to convert a Twist2D into left and right wheel velocities
     */
    public static DriveSignal inverseKinematics(Twist2D velocity) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveSignal(velocity.dx, velocity.dx);
        }
        double delta_v = Constants.kDriveWheelTrackWidthInches * velocity.dtheta / (2 * Constants.kTrackScrubFactor);
        return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}