package frc.lib.motion_profile;

/**
 * Constraints for constructing a MotionProfile.
 * includes the  maximum allowed velocity and maximum allowed acceleration.
 */
public class MaxVel_MaxAcc {
    protected double max_abs_vel = Double.POSITIVE_INFINITY;
    protected double max_abs_acc = Double.POSITIVE_INFINITY;

    public MaxVel_MaxAcc(double max_vel, double max_acc) {
        this.max_abs_vel = Math.abs(max_vel);
        this.max_abs_acc = Math.abs(max_acc);
    }

    /**
     * @return The (positive) maximum allowed velocity.
     */
    public double getMaxVel() {
        return max_abs_vel;
    }

    /**
     * @return The (positive) maximum allowed acceleration.
     */
    public double getMaxAcc() {
        return max_abs_acc;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MaxVel_MaxAcc)) {
            return false;
        }

        final MaxVel_MaxAcc other = (MaxVel_MaxAcc) obj;
        return (other.getMaxAcc() == getMaxAcc()) && (other.getMaxVel() == getMaxVel());
    }
}
