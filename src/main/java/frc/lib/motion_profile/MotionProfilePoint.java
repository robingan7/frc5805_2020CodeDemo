package frc.lib.motion_profile;

import static frc.lib.utility.Utli.EPSILON;
import static frc.lib.utility.Utli.epsilonEquals;

/**
 * Represent a point on motion profile graph (Trapezoid graph in this case)
 * A MotionProfilePoint is a completely specified state of 1D motion through time.
 */
public class MotionProfilePoint {
    protected final double t; //time - x coordinate
    protected final double pos;//position - intergral of the graph
    protected final double vel;//velocity - y coordinate
    protected final double acc;//acceleration - derivative of the graph

    public static MotionProfilePoint kInvalidState = new MotionProfilePoint(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

    public MotionProfilePoint(double t, double pos, double vel, double acc) {
        this.t = t;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }

    public MotionProfilePoint(MotionProfilePoint state) {
        this(state.t, state.pos, state.vel, state.acc);
    }

    public double getTime() {
        return t;
    }

    public double getPosition() {
        return pos;
    }

    public double getVelocity() {
        return vel;
    }

    public double getVelocitySquare() {
        return Math.pow(vel, 2);
    }

    public double getAcceleration() {
        return acc;
    }

    /**
     * Extrapolates this MotionProfilePoint to the specified time by applying this MotionProfilePoint's acceleration.
     *
     * @param t The time of the new MotionProfilePoint.
     * @return A MotionProfilePoint that is a valid predecessor (if t<=0) or successor (if t>=0) of this state.
     */
    public MotionProfilePoint extrapolate(double t) {
        return extrapolate(t, acc);
    }

    /**
     * Extrapolates this MotionProfilePoint to the specified time by applying a given acceleration to the (t, pos, vel) portion
     * of this MotionProfilePoint.
     * 
     * Basically get the next point
     *
     * @param t   The time of the new MotionProfilePoint.
     * @param acc The acceleration to apply.
     * @return A MotionProfilePoint that is a valid predecessor (if t<=0) or successor (if t>=0) of this state (with the
     * specified accel).
     */
    public MotionProfilePoint extrapolate(double t, double acc) {
        final double dt = t - this.t;
        return new MotionProfilePoint(t, pos + vel * dt + .5 * acc * dt * dt, vel + acc * dt, acc);
        //vel * dt + .5 * acc * dt * dt - increment of position
        //acc * dt - increment of speed
        //@see AP Physics 1 equation sheet
    }

    /**
     * Find the next time (first time > MotionProfilePoint.t()) that this MotionProfilePoint will be at pos. This is an inverse of the
     * extrapolate() method.
     *
     * @param pos The position to query.
     * @return The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never reach pos.
     */
    public double nextTimeAtPos(double pos) {
        if (epsilonEquals(pos, this.pos, EPSILON)) {
            // Already at pos.
            return t;
        }
        if (epsilonEquals(acc, 0.0, EPSILON)) {
            // Zero acceleration case.
            final double delta_pos = pos - this.pos;

            //Math.signnum - turns the Sign function of a value passed to it as argument. 
            if (!epsilonEquals(vel, 0.0, EPSILON) && Math.signum(delta_pos) == Math.signum(vel)) {

                // Constant velocity heading towards pos.
                return delta_pos / vel + t;
            }
            return Double.NaN;
        }

        // Solve the quadratic formula.
        // ax^2 + bx + c == 0
        // x = dt
        // a = .5 * acc
        // b = vel
        // c = this.pos - pos

        //next velocity
        final double disc = vel * vel - 2.0 * acc * (this.pos - pos);//@see AP Physics 1 equation sheet
        
        if (disc < 0.0) {
            // Extrapolating this MotionProfilePoint never reaches the desired pos.
            return Double.NaN;
        }

        final double sqrt_disc = Math.sqrt(disc);
        final double max_dt = (-vel + sqrt_disc) / acc;
        final double min_dt = (-vel - sqrt_disc) / acc;

        if (min_dt >= 0.0 && (max_dt < 0.0 || min_dt < max_dt)) {
            return t + min_dt;
        }
        if (max_dt >= 0.0) {
            return t + max_dt;
        }
        // We only reach the desired pos in the past.
        return Double.NaN;
    }

    @Override
    public String toString() {
        return "(t=" + t + ", pos=" + pos + ", vel=" + vel + ", acc=" + acc + ")";
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a nominal tolerance).
     */
    @Override
    public boolean equals(Object other) {
        return (other instanceof MotionProfilePoint) && equals((MotionProfilePoint) other, EPSILON);
    }

    /**
     * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified tolerance).
     */
    public boolean equals(MotionProfilePoint other, double epsilon) {
        return coincident(other, epsilon) && epsilonEquals(acc, other.acc, epsilon);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration
     * may be different).
     */
    public boolean coincident(MotionProfilePoint other) {
        return coincident(other, EPSILON);
    }

    /**
     * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified tolerance, but
     * acceleration may be different).
     */
    public boolean coincident(MotionProfilePoint other, double epsilon) {
        return epsilonEquals(t, other.t, epsilon) && epsilonEquals(pos, other.pos, epsilon)
                && epsilonEquals(vel, other.vel, epsilon);
    }

    /**
     * Returns a MotionProfilePoint that is the mirror image of this one. Pos, vel, and acc are all negated, but time is not.
     */
    public MotionProfilePoint flipped() {
        return new MotionProfilePoint(t, -pos, -vel, -acc);
    }
}
