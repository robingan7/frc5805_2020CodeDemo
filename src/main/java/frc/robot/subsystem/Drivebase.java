package frc.robot.subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.lib.waypoint.*;

import frc.lib.utility.DriveSignal;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.cycle.*;
import frc.lib.motor.MotorUtility;
import frc.robot.Constants;
import frc.robot.cycle.Subsystem_Cycle;
import frc.lib.path.Path;
import frc.lib.path.Lookahead;
import frc.lib.path.PathFollower;
import frc.robot.statesAndMachanics.RobotState;
import frc.robot.statesAndMachanics.Kinematics;

public class Drivebase extends Subsystem_Cycle{
    private final WPI_TalonSRX leftMaster_, rightMaster_;
    private final WPI_VictorSPX leftSlaveA_, rightSlaveA_, leftSlaveB_, rightSlaveB_;

    private static Drivebase instance_;
    private FeedData feedData_;
    private PigeonIMU pigeonIMU_;
    private DifferentialDrive telep_drive;
    private final Solenoid gearShifter_;
    private final Solenoid frontLifter_;
    private final Solenoid backLifter_;

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    private Rotation2D mGyroOffset = Rotation2D.getDefault();

    private DriveControlState driveControlState_;
    private boolean isBrakeMode_, isHighGear_, isFrontLifted_, isBackLifted_, autoShift_;

    private double lastActiveTime = 0;

     // Controllers
     private PathFollower pathFollower_;
     private Path currentPath_ = null;

    private final Cycle cycle_ = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drivebase.this) {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drivebase.this) {
                switch (driveControlState_) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        if (pathFollower_ != null) {
                            updatePathFollower(timestamp);
                        }
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + driveControlState_);
                        break;
                }
                //gearShifter_.set(false);
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
            //stopLogging();
        }
    };


    public static Drivebase getInstance(){
        if (instance_ == null) {
            instance_ = new Drivebase();
        }

        return instance_;
    }

    public Drivebase(){
        feedData_= new FeedData();
         // Start all Talons in open loop mode.
         leftMaster_ = MotorUtility.createTalon( Constants.kLeftDriveMasterId);
         configureMaster(leftMaster_, true);
 
         leftSlaveA_ = MotorUtility.createVictor(Constants.kLeftDriveSlaveAId,
                 Constants.kLeftDriveMasterId);
         leftSlaveA_.setInverted(false);
 
         leftSlaveB_ = MotorUtility.createVictor(Constants.kLeftDriveSlaveBId,
                 Constants.kLeftDriveMasterId);
         leftSlaveB_.setInverted(false);

 
         rightMaster_ = MotorUtility.createTalon(Constants.kRightDriveMasterId);
         configureMaster(rightMaster_, true);
 
         rightSlaveA_ = MotorUtility.createVictor(Constants.kRightDriveSlaveAId,
                 Constants.kRightDriveMasterId);
         //rightSlaveA_.setInverted(true);
 
         rightSlaveB_ = MotorUtility.createVictor(Constants.kRightDriveSlaveBId,
                 Constants.kRightDriveMasterId);
         //rightSlaveB_.setInverted(true);
 
         pigeonIMU_ = new PigeonIMU(leftMaster_);
        //It was a slave motor in 254's code

        telep_drive = new DifferentialDrive(new SpeedControllerGroup(leftMaster_, leftSlaveA_,leftSlaveB_), 
                                        new SpeedControllerGroup(rightMaster_, rightSlaveA_,rightSlaveB_));

         gearShifter_ = new Solenoid(Constants.kGearShifter);
         frontLifter_ = new Solenoid(Constants.kFrontLifter);
         backLifter_ = new Solenoid(Constants.kBackLifter);

         isHighGear_ = false;
         isBackLifted_ = false;
         isFrontLifted_ = false;

         resetGains();
    }
    
    public synchronized void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            isBrakeMode_ = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            rightMaster_.setNeutralMode(mode);
            rightSlaveA_.setNeutralMode(mode);
            rightSlaveB_.setNeutralMode(mode);

            leftMaster_.setNeutralMode(mode);
            leftSlaveA_.setNeutralMode(mode);
            leftSlaveB_.setNeutralMode(mode);
        }
    }

    public synchronized void setHighGear() {
        boolean old_value = isHighGear_;
        isHighGear_ = !old_value;

        gearShifter_.set(isHighGear_);
    }

    public boolean isAbleActive(){
        return Math.abs(lastActiveTime - Timer.getFPGATimestamp()) > 0.9;
    }

    
    public synchronized boolean isDoneWithPath() {
        if (driveControlState_ == DriveControlState.PATH_FOLLOWING && pathFollower_ != null) {
            return pathFollower_.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void setFrontLifter() {
        if(isAbleActive()){
            boolean old_value = isFrontLifted_;
            isFrontLifted_ = !old_value;
            frontLifter_.set(isFrontLifted_);

            lastActiveTime = Timer.getFPGATimestamp();
        }
    }

    public synchronized void setBackLifter() {
        if(isAbleActive()){
            boolean old_value = isBackLifted_;
            isBackLifted_ = !old_value;
            
            backLifter_.set(isBackLifted_);

            lastActiveTime = Timer.getFPGATimestamp();
        }
    }

    private void configureMaster(WPI_TalonSRX talon, boolean left) {
        /*
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }*/
        talon.setInverted(!left);
    }

    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public synchronized void resetEncoders() {
        leftMaster_.setSelectedSensorPosition(0, 0, 0);
        rightMaster_.setSelectedSensorPosition(0, 0, 0);
        feedData_ = new FeedData();
    }

    public void resetSensors() {
        setHeading(Rotation2D.getDefault());
        resetEncoders();
        autoShift_ = true;

        gearShifter_.set(false);
        frontLifter_.set(false);
        backLifter_.set(false);
    }

    public synchronized void setHeading(Rotation2D heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        //mGyroOffset = heading.rotateFromAnother(Rotation2D.fromAngle(mPigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        feedData_.gyro_heading = heading;
    }

     /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (currentPath_ != path || driveControlState_ != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            pathFollower_ = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed),
                    Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                    Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                    Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                    Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                    Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                    Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            driveControlState_ = DriveControlState.PATH_FOLLOWING;
            currentPath_ = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            autoShift_ = true;
            /*
            System.out.println("Switching to open loop");
            System.out.println(signal);
            driveControlState_ = DriveControlState.OPEN_LOOP;
            leftMaster_.configNeutralDeadband(0.04, 0);
            rightMaster_.configNeutralDeadband(0.04, 0);*/
        }
        feedData_.left_demand = 0.0;
        feedData_.right_demand = 0.0;

        feedData_.xspeed = signal.getSpeed();
        feedData_.zrotation = signal.getRotation();

        feedData_.left_feedforward = 0.0;
        feedData_.right_feedforward = 0.0;
    }

    private void updatePathFollower(double timestamp) {
        if (driveControlState_ == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2D field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2D command = pathFollower_.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!pathFollower_.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if (!pathFollower_.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized Rotation2D getHeading() {
        return feedData_.gyro_heading;
    }

    public void registerEnabledLoops(ICycle_in regist) {
        regist.addSubsystem(cycle_);
    }

    /**
     * this is the autonomous feeding method
     * @param signal
     * @param feedforward
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING) {
            driveControlState_ = DriveControlState.PATH_FOLLOWING;
            setBrakeMode(true);
            autoShift_ = false;
            
            /*
            leftMaster_.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            rightMaster_.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            leftMaster_.configNeutralDeadband(0.0, 0);
            rightMaster_.configNeutralDeadband(0.0, 0);*/
        }
        feedData_.left_demand = signal.getLeft();
        feedData_.right_demand = signal.getRight();
        feedData_.left_feedforward = feedforward.getLeft();
        feedData_.right_feedforward = feedforward.getRight();
    }

    public synchronized void forceDoneWithPath() {
        if (driveControlState_ == DriveControlState.PATH_FOLLOWING && pathFollower_ != null) {
            pathFollower_.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (driveControlState_ == DriveControlState.PATH_FOLLOWING && pathFollower_ != null) {
            return pathFollower_.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    public synchronized void resetGains() {
        leftMaster_.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        leftMaster_.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        leftMaster_.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        leftMaster_.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        leftMaster_.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        rightMaster_.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        rightMaster_.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        rightMaster_.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        rightMaster_.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        rightMaster_.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public enum DriveCurrentLimitState {
        UNTHROTTLED, THROTTLED
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    public static class FeedData {
        // Motion Profile Feedback
        public double timestamp;
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2D gyro_heading = Rotation2D.getDefault();
        public Pose2D error = Pose2D.getDefault();
        public double left_voltage;
        public double right_voltage;

        // Motion Profile Feedforward 
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;

        //telep input value
        public double xspeed;
        public double zrotation;
        public double left_demand;
        public double right_demand;
        //public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.getDefault());
    }

    /**
     * override from iSubsystem
     * move the drive base depends the state(telep or autonomous)
     */
    @Override
    public synchronized void move_subsystem(){
        //System.out.println(driveControlState_);
        if (driveControlState_ == DriveControlState.OPEN_LOOP) {
            telep_drive.arcadeDrive(feedData_.xspeed, feedData_.zrotation);
            //System.out.println(feedData_.xspeed + " " + feedData_.zrotation);
        } else {
            leftMaster_.set(ControlMode.Velocity, feedData_.left_demand, DemandType.ArbitraryFeedForward,
                    feedData_.left_feedforward + Constants.kDriveLowGearVelocityKd * feedData_.left_accel / 1023.0);
            rightMaster_.set(ControlMode.Velocity, feedData_.right_demand, DemandType.ArbitraryFeedForward,
                    feedData_.right_feedforward + Constants.kDriveLowGearVelocityKd * feedData_.right_accel / 1023.0);
            System.out.println("left: " + feedData_.left_demand + " || right: " + feedData_.right_demand);
        }
    }

    @Override
    public synchronized void update_subsystem(){
        feedData_.timestamp = Timer.getFPGATimestamp();
        double prevLeftTicks = feedData_.left_position_ticks;
        double prevRightTicks = feedData_.right_position_ticks;

        /*
        feedData_.left_voltage = leftMaster_.getAppliedOutput() * leftMaster_.getBusVoltage();
        feedData_.right_voltage = rightMaster_.getAppliedOutput() * rightMaster_.getBusVoltage();

        feedData_.left_position_ticks = mLeftEncoder.get();
        feedData_.right_position_ticks = mRightEncoder.get();
        feedData_.gyro_heading = Rotation2d.fromDegrees(pigeonIMU_.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((feedData_.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        feedData_.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

        double deltaRightTicks = ((feedData_.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        feedData_.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

        feedData_.left_velocity_ticks_per_100ms = (int) (mLeftEncoder.getRate()
                / (10 * mLeftEncoder.getDistancePerPulse()));
        feedData_.right_velocity_ticks_per_100ms = (int) (mRightEncoder.getRate()
                / (10 * mRightEncoder.getDistancePerPulse()));*/
    }

    @Override 
    public void sendDataToSmartDashboard(){
        SmartDashboard.putNumber("right: ", rightMaster_.getSelectedSensorPosition());
        SmartDashboard.putNumber("left: ", leftMaster_.getSelectedSensorPosition());
    }

    @Override
    public boolean checkSubsystem() {
        return true;
    }
}