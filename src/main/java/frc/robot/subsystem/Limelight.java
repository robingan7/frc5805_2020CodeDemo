package frc.robot.subsystem;

import frc.robot.Constants;
import frc.robot.statesAndMachanics.RobotState;
import frc.lib.waypoint.Rotation2D;
import frc.lib.waypoint.Pose2D;
import frc.lib.waypoint.Translation2D;
import frc.lib.utility.Utility;
import frc.lib.vision.TargetInfo;
import frc.robot.cycle.Subsystem_Cycle;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem_Cycle {
    public final static int kDefaultPipeline = 0;
    public final static int kSortTopPipeline = 1;

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Pose2D kTurretToLens = Pose2D.getDefault();
        public Rotation2D kHorizontalPlaneToLens = Rotation2D.getDefault();
    }

    private NetworkTable mNetworkTable;

    public Limelight(LimelightConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
    }

    public static class FeedData {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConstants mConstants = null;
    private FeedData feedData_ = new FeedData();
    private boolean mOutputsHaveChanged = true;
    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private List<TargetInfo> mTargets = new ArrayList<>();
    private boolean mSeesTarget = false;

    public Pose2D getTurretToLens() {
        return mConstants.kTurretToLens;
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2D getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    @Override
    public synchronized void update_subsystem() {
        feedData_.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
        feedData_.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        feedData_.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        feedData_.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        feedData_.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        feedData_.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void move_subsystem() {
        if (feedData_.givenLedMode != feedData_.ledMode ||
                feedData_.givenPipeline != feedData_.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(feedData_.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(feedData_.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(feedData_.pipeline);
            mNetworkTable.getEntry("stream").setNumber(feedData_.stream);
            mNetworkTable.getEntry("snapshot").setNumber(feedData_.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSubsystem(){
        return true;
    }

    @Override
    public synchronized void sendDataToSmartDashboard() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", feedData_.latency);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != feedData_.ledMode) {
            feedData_.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != feedData_.pipeline) {
            RobotState.getInstance().resetVision();
            feedData_.pipeline = mode;

            System.out.println(feedData_.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return feedData_.pipeline;
    }

    public synchronized boolean seesTarget() {
        return mSeesTarget;
    }

    /**
     * @return two targets that make up one hatch/port or null if less than two targets are found
     */
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if (seesTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Utility.EPSILON) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) /
                    (corners.get(1)[0] - corners.get(0)[0]);
        }

        mTargets.clear();
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);

            double y = Constants.kVPW / 2 * nY;
            double z = Constants.kVPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            mTargets.add(target);
        }

        return mTargets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] xCorners = mNetworkTable.getEntry("tcornx").getDoubleArray(mZeroArray);
        double[] yCorners = mNetworkTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget ||
                Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
                || xCorners.length != 8 || yCorners.length != 8) {
            return null;
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private static final Comparator<Translation2D> xSort = Comparator.comparingDouble(Translation2D::x);
    private static final Comparator<Translation2D> ySort = Comparator.comparingDouble(Translation2D::y);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2D> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2D(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2D> left = corners.subList(0, 4);
        List<Translation2D> right = corners.subList(4, 8);

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2D> leftTop = left.subList(0, 2);
        List<Translation2D> rightTop = right.subList(0, 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2D leftCorner = leftTop.get(0);
        Translation2D rightCorner = rightTop.get(1);

        return List.of(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }

    public double getLatency() {
        return feedData_.latency;
    }
}