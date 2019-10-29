package frc.lib.motion_profile;

import static frc.lib.utility.Utility.EPSILON;
import static frc.lib.utility.Utility.epsilonEquals;

/**
 * A MotionProfileSegment is a movement from a start MotionProfilePoint to an end MotionProfilePoint with a constant acceleration.
 * main functin is isValid()
 */
public class MotionProfileSegment {
    protected MotionProfilePoint mStart;
    protected MotionProfilePoint mEnd;

    public MotionProfileSegment(MotionProfilePoint start, MotionProfilePoint end) {
        mStart = start;
        mEnd = end;
    }

    /**
     * Verifies that:
     * <p>
     * 1. All segments have a constant acceleration.
     * <p>
     * 2. All segments have monotonic position (sign of velocity doesn't change).
     * <p>
     * 3. The time, position, velocity, and acceleration of the profile are consistent.
     */
    public boolean isValid() {
        if (!epsilonEquals(getStartPoint().getAcceleration(), getEndPoint().getAcceleration(), EPSILON)) {
            // Acceleration is not constant within the segment.
            System.err.println(
                    "Segment acceleration not constant! Start acc: " + getStartPoint().getAcceleration() + ", End acc: " + getEndPoint().getAcceleration());
            return false;
        }

        if (Math.signum(getStartPoint().getVelocity()) * Math.signum(getEndPoint().getVelocity()) < 0.0 && !epsilonEquals(getStartPoint().getVelocity(), 0.0, EPSILON)
                && !epsilonEquals(getEndPoint().getVelocity(), 0.0, EPSILON)) {
            // Velocity direction reverses within the segment.
            System.err.println("Segment velocity reverses! Start vel: " + getStartPoint().getVelocity() + ", End vel: " + getEndPoint().getVelocity());
            return false;
        }

        if (!getStartPoint().extrapolate(getEndPoint().getTime()).equals(getEndPoint())) {
            // A single segment is not consistent.
            if (getStartPoint().getTime() == getEndPoint().getTime() && Double.isInfinite(getStartPoint().getAcceleration())) {
                // One allowed exception: If acc is infinite and dt is zero.
                return true;
            }
            System.err.println("Segment not consistent! Start: " + getStartPoint() + ", End: " + getEndPoint());
            return false;
        }
        return true;
    }

    public boolean includesTime(double t) {
        return t >= getStartPoint().getTime() && t <= getEndPoint().getTime();
    }

    public boolean includesPos(double pos) {
        return pos >= getStartPoint().getPosition() && pos <= getEndPoint().getPosition() || pos <= getStartPoint().getPosition() && pos >= getEndPoint().getPosition();
    }

    public MotionProfilePoint getStartPoint() {
        return mStart;
    }

    public void setStart(MotionProfilePoint start) {
        mStart = start;
    }

    public MotionProfilePoint getEndPoint() {
        return mEnd;
    }

    public void setEnd(MotionProfilePoint end) {
        mEnd = end;
    }

    @Override
    public String toString() {
        return "Start: " + getStartPoint() + ", End: " + getEndPoint();
    }
}
