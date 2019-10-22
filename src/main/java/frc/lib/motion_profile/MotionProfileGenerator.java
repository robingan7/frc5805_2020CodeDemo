package frc.lib.motion_profile;

import frc.lib.motion_profile.MotionProfileGoal.CompletionBehavior;

/**
 * A MotionProfileGenerator generates minimum-time MotionProfiles to travel from a given MotionProfilePoint to a given
 * MotionProfileGoal while obeying a set of MaxVel_MaxAcc.
 */
public class MotionProfileGenerator {
    // Static class.
    private MotionProfileGenerator() {}

    protected static MotionProfile generateFlippedProfile(MaxVel_MaxAcc constraints,
                                                          MotionProfileGoal goal_state, MotionProfilePoint prev_state) {
        MotionProfile profile = generateProfile(constraints, goal_state.flipped(), prev_state.flipped());
        for (MotionProfileSegment s : profile.segments()) {
            s.setStart(s.getStartPoint().flipped());
            s.setEnd(s.getEndPoint().flipped());
        }
        return profile;
    }

    /**
     * Generate a motion profile.
     *
     * @param constraints The constraints to use.
     * @param goal_state  The goal to use.
     * @param prev_state  The initial state to use.
     * @return A motion profile from prev_state to goal_state that satisfies constraints.
     */
    public synchronized static MotionProfile generateProfile(MaxVel_MaxAcc constraints,
                                                             MotionProfileGoal goal_state,
                                                             MotionProfilePoint prev_state) {
        double delta_pos = goal_state.getPosition() - prev_state.getPosition();

        if (delta_pos < 0.0 || (delta_pos == 0.0 && prev_state.getVelocity() < 0.0)) {
            // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
            // flip the solution.
            return generateFlippedProfile(constraints, goal_state, prev_state);
        }

        // Invariant from this point on: delta_pos >= 0.0
        // Clamp the start state to be valid.
        MotionProfilePoint start_state = new MotionProfilePoint(prev_state.getTime(), prev_state.getPosition(),
                Math.signum(prev_state.getVelocity()) * Math.min(Math.abs(prev_state.getVelocity()), constraints.getMaxVel()),
                Math.signum(prev_state.getAcceleration()) * Math.min(Math.abs(prev_state.getAcceleration()), constraints.getMaxAcc()));

        MotionProfile profile = new MotionProfile();
        profile.reset(start_state);

        // If our velocity is headed away from the goal, the first thing we need to do is to stop.
        if (start_state.getVelocity() < 0.0 && delta_pos > 0.0) {
            final double stopping_time = Math.abs(start_state.getVelocity() / constraints.getMaxAcc());
            profile.appendControl(constraints.getMaxAcc(), stopping_time);
            start_state = profile.endState();
            delta_pos = goal_state.getPosition() - start_state.getPosition();
        }

        // Invariant from this point on: start_state.getVelocity() >= 0.0
        final double min_abs_vel_at_goal_sqr = start_state.getVelocitySquare() - 2.0 * constraints.getMaxAcc() * delta_pos;
        final double min_abs_vel_at_goal = Math.sqrt(Math.abs(min_abs_vel_at_goal_sqr));
        final double max_abs_vel_at_goal = Math.sqrt(start_state.getVelocitySquare() + 2.0 * constraints.getMaxAcc() * delta_pos);

        double goal_vel = goal_state.getMaxVel();
        double max_acc = constraints.getMaxAcc();

        if (min_abs_vel_at_goal_sqr > 0.0
                && min_abs_vel_at_goal > (goal_state.getMaxVel() + goal_state.getVelocityTolerance())) {

            // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
            // do.
            if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ABS_VEL) {

                // Adjust the goal velocity.
                goal_vel = min_abs_vel_at_goal;
            } else if (goal_state.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ACCEL) {
                if (Math.abs(delta_pos) < goal_state.getPositionTolerance()) {

                    // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                    // which will result in NaNs below, so we can return the profile immediately.
                    profile.appendSegment(new MotionProfileSegment(
                            new MotionProfilePoint(profile.endTime(), profile.endPos(), profile.endState().getVelocity(),
                                    Double.NEGATIVE_INFINITY),
                            new MotionProfilePoint(profile.endTime(), profile.endPos(), goal_vel, Double.NEGATIVE_INFINITY)));

                    profile.consolidate();
                    return profile;
                }

                // Adjust the max acceleration.
                max_acc = Math.abs(goal_vel * goal_vel - start_state.getVelocitySquare()) / (2.0 * delta_pos);
            } else {

                // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                final double stopping_time = Math.abs(start_state.getVelocity() / constraints.getMaxAcc());
                profile.appendControl(-constraints.getMaxAcc(), stopping_time);

                // Now we need to travel backwards, so generate a flipped profile.
                profile.appendProfile(generateFlippedProfile(constraints, goal_state, profile.endState()));
                profile.consolidate();
                return profile;
            }
        }
        goal_vel = Math.min(goal_vel, max_abs_vel_at_goal);

        // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
        // max_acc.

        // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
        // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
        // is greater than constraints.max_abs_vel, we will clamp and cruise.
        // Solve the following three equations to find Vmax (by substitution):
        // Vmax^2 = Vstart^2 + 2*a*d_accel
        // Vgoal^2 = Vmax^2 - 2*a*d_decel
        // delta_pos = d_accel + d_decel
        final double v_max = Math.min(constraints.getMaxVel(),
                Math.sqrt((start_state.getVelocitySquare() + goal_vel * goal_vel) / 2.0 + delta_pos * max_acc));

        // Accelerate to v_max
        if (v_max > start_state.getVelocity()) {
            final double accel_time = (v_max - start_state.getVelocity()) / max_acc;
            profile.appendControl(max_acc, accel_time);
            start_state = profile.endState();
        }
        // Figure out how much distance will be covered during deceleration.
        final double distance_decel = Math.max(0.0,
                (start_state.getVelocitySquare() - goal_vel * goal_vel) / (2.0 * constraints.getMaxAcc()));
                
        final double distance_cruise = Math.max(0.0, goal_state.getPosition() - start_state.getPosition() - distance_decel);
        // Cruise at constant velocity.
        if (distance_cruise > 0.0) {
            final double cruise_time = distance_cruise / start_state.getVelocity();
            profile.appendControl(0.0, cruise_time);
            start_state = profile.endState();
        }
        // Decelerate to goal velocity.
        if (distance_decel > 0.0) {
            final double decel_time = (start_state.getVelocity() - goal_vel) / max_acc;
            profile.appendControl(-max_acc, decel_time);
        }

        profile.consolidate();
        return profile;
    }
}
