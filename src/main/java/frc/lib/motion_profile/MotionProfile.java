package frc.lib.motion_profile;

import static frc.lib.utility.Utility.EPSILON;
import static frc.lib.utility.Utility.epsilonEquals;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

/**
 * A motion profile specifies a 1D time-parameterized trajectory. The trajectory is composed of successively coincident
 * MotionSegments from which the desired state of motion at any given distance or time can be calculated.
 */
public class MotionProfile {
    protected List<MotionProfileSegment> mSegments;

    /**
     * Create an empty MotionProfile.
     */
    public MotionProfile() {
        mSegments = new ArrayList<>();
    }

    /**
     * Create a MotionProfile from an existing list of segments (note that validity is not checked).
     *
     * @param segments The new segments of the profile.
     */
    public MotionProfile(List<MotionProfileSegment> segments) {
        mSegments = segments;
    }

    /**
     * Checks if the given MotionProfile is valid. This checks that:
     * <p>
     * 1. All segments are valid.
     * <p>
     * 2. Successive segments are C1 continuous in position and C0 continuous in velocity.
     *
     * @return True if the MotionProfile is valid.
     */
    public boolean isValid() {
        MotionProfileSegment prev_segment = null;
        for (MotionProfileSegment s : mSegments) {
            if (!s.isValid()) {
                return false;
            }
            if (prev_segment != null && !s.getStartPoint().coincident(prev_segment.getEndPoint())) {
                // Adjacent segments are not continuous.
                System.err.println("Segments not continuous! End: " + prev_segment.getEndPoint() + ", Start: " + s.getStartPoint());
                return false;
            }
            prev_segment = s;
        }
        return true;
    }

    /**
     * Check if the profile is empty.
     *
     * @return True if there are no segments.
     */
    public boolean isEmpty() {
        return mSegments.isEmpty();
    }

    /**
     * Get the interpolated MotionProfilePoint at any given time.
     *
     * @param t The time to query.
     * @return Empty if the time is outside the time bounds of the profile, or the resulting MotionProfilePoint otherwise.
     */
    public Optional<MotionProfilePoint> stateByTime(double t) {
        if (t < startTime() && t + EPSILON >= startTime()) {
            return Optional.of(startState());
        }

        if (t > endTime() && t - EPSILON <= endTime()) {
            return Optional.of(endState());
        }

        for (MotionProfileSegment s : mSegments) {
            if (s.includesTime(t)) {
                return Optional.of(s.getStartPoint().extrapolate(t));
            }
        }

        return Optional.empty();
    }

    /**
     * Get the interpolated MotionProfilePoint at any given time, clamping to the endpoints if time is out of bounds.
     *
     * @param t The time to query.
     * @return The MotionProfilePoint at time t, or closest to it if t is outside the profile.
     */
    public MotionProfilePoint stateByTimeClamped(double t) {
        if (t < startTime()) {
            return startState();
        } else if (t > endTime()) {
            return endState();
        }

        for (MotionProfileSegment s : mSegments) {
            if (s.includesTime(t)) {
                return s.getStartPoint().extrapolate(t);
            }
        }
        // Should never get here.
        return MotionProfilePoint.kInvalidState;
    }

    /**
     * Get the interpolated MotionProfilePoint by distance (the "getPosition()" field of MotionProfilePoint). Note that since a profile may
     * reverse, this method only returns the *first* instance of this position.
     *
     * @param pos The position to query.
     * @return Empty if the profile never crosses pos or if the profile is invalid, or the resulting MotionProfilePoint
     * otherwise.
     */
    public Optional<MotionProfilePoint> firstStateByPos(double pos) {
        for (MotionProfileSegment s : mSegments) {
            if (s.includesPos(pos)) {
                if (epsilonEquals(s.getEndPoint().getPosition(), pos, EPSILON)) {
                    return Optional.of(s.getEndPoint());
                }

                final double t = Math.min(s.getStartPoint().nextTimeAtPos(pos), s.getEndPoint().getTime());

                if (Double.isNaN(t)) {
                    System.err.println("Error! We should reach 'pos' but we don't");
                    return Optional.empty();
                }
                return Optional.of(s.getStartPoint().extrapolate(t));
            }
        }
        // We never reach pos.
        return Optional.empty();
    }

    /**
     * Remove all parts of the profile prior to the query time. This eliminates whole segments and also shortens any
     * segments containing t.
     *
     * @param t The query time.
     */
    public void trimBeforeTime(double t) {
        for (Iterator<MotionProfileSegment> iterator = mSegments.iterator(); iterator.hasNext(); ) {
            MotionProfileSegment s = iterator.next();
            if (s.getEndPoint().getTime() <= t) {
                // Segment is fully before t.
                iterator.remove();
                continue;
            }
            if (s.getStartPoint().getTime() <= t) {
                // Segment begins before t; let's shorten the segment.
                s.setStart(s.getStartPoint().extrapolate(t));
            }
            break;
        }
    }

    /**
     * Remove all segments.
     */
    public void clear() {
        mSegments.clear();
    }

    /**
     * Remove all segments and initialize to the desired state (actually a segment of length 0 that starts and ends at
     * initial_state).
     *
     * @param initial_state The MotionProfilePoint to initialize to.
     */
    public void reset(MotionProfilePoint initial_state) {
        clear();
        mSegments.add(new MotionProfileSegment(initial_state, initial_state));
    }

    /**
     * Remove redundant segments (segments whose start and end states are coincident).
     */
    public void consolidate() {
        for (Iterator<MotionProfileSegment> iterator = mSegments.iterator(); iterator.hasNext() && mSegments.size() > 1; ) {
            MotionProfileSegment s = iterator.next();

            if (s.getStartPoint().coincident(s.getEndPoint())) {
                iterator.remove();
            }
        }
    }

    /**
     * Add to the profile by applying an acceleration control for a given time. This is appended to the previous last
     * state.
     *
     * @param acc The acceleration to apply.
     * @param dt  The period of time to apply the given acceleration.
     */
    public void appendControl(double acc, double dt) {
        if (isEmpty()) {
            System.err.println("Error!  Trying to append to empty profile");
            return;
        }
        MotionProfilePoint last_end_state = mSegments.get(mSegments.size() - 1).getEndPoint();
        MotionProfilePoint new_start_state = new MotionProfilePoint(last_end_state.getTime(), last_end_state.getPosition(), last_end_state.getVelocity(),
                acc);

        appendSegment(new MotionProfileSegment(new_start_state, new_start_state.extrapolate(new_start_state.getTime() + dt)));
    }

    /**
     * Add to the profile by inserting a new segment. No validity checking is done.
     *
     * @param segment The segment to add.
     */
    public void appendSegment(MotionProfileSegment segment) {
        mSegments.add(segment);
    }

    /**
     * Add to the profile by inserting a new profile after the final state. No validity checking is done.
     *
     * @param profile The profile to add.
     */
    public void appendProfile(MotionProfile profile) {
        for (MotionProfileSegment s : profile.segments()) {
            appendSegment(s);
        }
    }

    /**
     * @return The number of segments.
     */
    public int size() {
        return mSegments.size();
    }

    /**
     * @return The list of segments.
     */
    public List<MotionProfileSegment> segments() {
        return mSegments;
    }

    /**
     * @return The first state in the profile (or kInvalidState if empty).
     */
    public MotionProfilePoint startState() {
        if (isEmpty()) {
            return MotionProfilePoint.kInvalidState;
        }
        return mSegments.get(0).getStartPoint();
    }

    /**
     * @return The time of the first state in the profile (or NaN if empty).
     */
    public double startTime() {
        return startState().getTime();
    }

    /**
     * @return The pos of the first state in the profile (or NaN if empty).
     */
    public double startPos() {
        return startState().getPosition();
    }

    /**
     * @return The last state in the profile (or kInvalidState if empty).
     */
    public MotionProfilePoint endState() {
        if (isEmpty()) {
            return MotionProfilePoint.kInvalidState;
        }
        return mSegments.get(mSegments.size() - 1).getEndPoint();
    }

    /**
     * @return The time of the last state in the profile (or NaN if empty).
     */
    public double endTime() {
        return endState().getTime();
    }

    /**
     * @return The pos of the last state in the profile (or NaN if empty).
     */
    public double endPos() {
        return endState().getPosition();
    }

    /**
     * @return The duration of the entire profile.
     */
    public double duration() {
        return endTime() - startTime();
    }

    /**
     * @return The total distance covered by the profile. Note that distance is the sum of absolute distances of all
     * segments, so a reversing profile will count the distance covered in each direction.
     */
    public double length() {
        double length = 0.0;
        for (MotionProfileSegment s : segments()) {
            length += Math.abs(s.getEndPoint().getPosition() - s.getStartPoint().getPosition());
        }
        return length;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder("Profile:");
        for (MotionProfileSegment s : segments()) {
            builder.append("\n\t");
            builder.append(s);
        }
        return builder.toString();
    }
}