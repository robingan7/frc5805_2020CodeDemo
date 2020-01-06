package frc.robot.statesAndMachanics;

import frc.robot.statesAndMachanics.SuperStructureCommand;
import frc.robot.subsystem.Limelight;
import frc.lib.waypoint.*;
import frc.lib.vision.*;
import frc.lib.vision.GoalTracker.TrackReportComparator;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 100;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Camera frame: origin is the center of the Limelight imager relative to the
     * turret.
     *
     * 4. Goal frame: origin is the center of the vision target, facing outwards
     * along the normal. Also note that there can be multiple goal frames.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Measured by the turret encoder. This is a pure
     * rotation.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-goal: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2D or Rotation2D
    private InterpolatingTreeMap<InterpolatingDouble, Pose2D> field_to_vehicle_;
    private Twist2D vehicle_velocity_predicted_;
    private Twist2D vehicle_velocity_measured_;
    private MovingAverageTwist2D vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private GoalTracker vision_target_low_ = new GoalTracker();
    private GoalTracker vision_target_high_ = new GoalTracker();

    List<Translation2D> mCameraToVisionTargetPosesLow = new ArrayList<>();
    List<Translation2D> mCameraToVisionTargetPosesHigh = new ArrayList<>();

    private RobotState() {
        reset(0.0, Pose2D.getDefault(), Rotation2D.getDefault());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2D initial_field_to_vehicle,
                                   Rotation2D initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
    }

    public synchronized void reset(double start_time, Pose2D initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2D.getDefault();
        vehicle_velocity_measured_ = Twist2D.getDefault();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2D(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2D.getDefault(), Rotation2D.getDefault());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2D getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2D> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2D getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2D getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2D getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        vision_target_low_.reset();
        vision_target_high_.reset();
    }

    private Translation2D getCameraToVisionTargetPose(TargetInfo target, boolean high, Limelight source) {
        // Compensate for camera pitch
        Translation2D xz_plane_translation = new Translation2D(target.getX(), target.getZ()).rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = source.getLensHeight() - (high ? Constants.kPortTargetHeight : Constants.kHatchTargetHeight);
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2D angle = new Rotation2D(x, y, true);
            return new Translation2D(distance * angle.cos(), distance * angle.sin());
        }

        return null;
    }

    private void updatePortGoalTracker(double timestamp, List<Translation2D> cameraToVisionTargetPoses, GoalTracker tracker, Limelight source) {
        if (cameraToVisionTargetPoses.size() != 2 ||
                cameraToVisionTargetPoses.get(0) == null ||
                cameraToVisionTargetPoses.get(1) == null) return;
        Pose2D cameraToVisionTarget = Pose2D.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
                cameraToVisionTargetPoses.get(1), 0.5));

        //Pose2D fieldToVisionTarget = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToVisionTarget);
        //tracker.update(timestamp, List.of(new Pose2D(fieldToVisionTarget.getTranslation(), Rotation2D.getDefault())));
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        mCameraToVisionTargetPosesLow.clear();
        mCameraToVisionTargetPosesHigh.clear();

        if (observations == null || observations.isEmpty()) {
            vision_target_low_.update(timestamp, new ArrayList<>());
            vision_target_high_.update(timestamp, new ArrayList<>());
            return;
        }

        for (TargetInfo target : observations) {
            mCameraToVisionTargetPosesLow.add(getCameraToVisionTargetPose(target, false, source));
            mCameraToVisionTargetPosesHigh.add(getCameraToVisionTargetPose(target, true, source));
        }

        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesLow, vision_target_low_, source);
        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesHigh, vision_target_high_, source);
    }

    // use known field target orientations to compensate for inaccuracy, assumes robot starts pointing directly away
    // from and perpendicular to alliance wall
    private final double[] kPossibleTargetNormals = {0.0, 90.0, 180.0, 270.0, 30.0, 150.0, 210.0, 330.0};

    public synchronized Pose2D getFieldToVisionTarget(boolean highTarget) {
        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;

        if (!tracker.hasTracks()) {
            return null;
        }

        Pose2D fieldToTarget = tracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (Math.abs(normalPositive - possible) < Math.abs(normalPositive - normalClamped)) {
                normalClamped = possible;
            }
        }

        return new Pose2D(fieldToTarget.getTranslation(), Rotation2D.fromDegrees(normalClamped));
    }

    public synchronized Pose2D getVehicleToVisionTarget(double timestamp, boolean highTarget) {
        Pose2D fieldToVisionTarget = getFieldToVisionTarget(highTarget);

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    public synchronized Optional<AimingParameters> getAimingParameters(boolean highTarget, int prev_track_id, double max_track_age) {
        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                Constants.kTrackStabilityWeight,
                Constants.kTrackAgeWeight,
                Constants.kTrackSwitchingWeight,
                prev_track_id, timestamp);
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }
        Pose2D vehicleToGoal = getFieldToVehicle(timestamp).inverse().transformBy(report.field_to_target).transformBy(getVisionTargetToGoalOffset());

        AimingParameters params = new AimingParameters(vehicleToGoal,
                report.field_to_target,
                report.field_to_target.getRotation(),
                report.latest_timestamp, report.stability, report.id);
        return Optional.of(params);
    }

    public Pose2D getRobot() {
        return new Pose2D();
    }

    public synchronized Pose2D getVisionTargetToGoalOffset() {
        // if (SuperstructureCommands.isInCargoShipPosition() && EndEffector.getInstance().getObservedGamePiece() == GamePiece.BALL) {
        //     return Pose2D.fromTranslation(new Translation2D(-6.0, 0.0));
        // }

        return Pose2D.getDefault();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
    }
}