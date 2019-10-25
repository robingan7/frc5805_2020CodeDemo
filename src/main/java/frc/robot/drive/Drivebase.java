package frc.robot.drive;
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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.cycle.*;
import frc.lib.motor.MotorUtil;
import frc.robot.Constants;
import frc.robot.subsystem.Subsystem_Function;

public class Drivebase extends Subsystem_Function{
    private final WPI_TalonSRX leftMaster_, rightMaster_;
    private final WPI_VictorSPX leftSlaveA_, rightSlaveA_, leftSlaveB_, rightSlaveB_;

    private static Drivebase instance_;
    private FeedData feedData_;
    private PigeonIMU mPigeonIMU;
    private DifferentialDrive telep_drive;
    private final Solenoid gearShifter_;
    private final Solenoid frontLifter_;
    private final Solenoid backLifter_;

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    private Rotation2d mGyroOffset = Rotation2d.default_;

    private DriveControlState currentDriveState;
    private boolean isBrakeMode_, isHighGear_, isFrontLifted_, isBackLifted_, autoShift_;

    private double lastActiveTime = 0;

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
                switch (currentDriveState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        //updatePathFollower(); Hope we have that
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + currentDriveState);
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
         leftMaster_ = MotorUtil.createTalon( Constants.kLeftDriveMasterId);
         configureMaster(leftMaster_, true);
 
         leftSlaveA_ = MotorUtil.createVictor(Constants.kLeftDriveSlaveAId,
                 Constants.kLeftDriveMasterId);
         leftSlaveA_.setInverted(false);
 
         leftSlaveB_ = MotorUtil.createVictor(Constants.kLeftDriveSlaveBId,
                 Constants.kLeftDriveMasterId);
         leftSlaveB_.setInverted(false);

 
         rightMaster_ = MotorUtil.createTalon(Constants.kRightDriveMasterId);
         configureMaster(rightMaster_, true);
 
         rightSlaveA_ = MotorUtil.createVictor(Constants.kRightDriveSlaveAId,
                 Constants.kRightDriveMasterId);
         //rightSlaveA_.setInverted(true);
 
         rightSlaveB_ = MotorUtil.createVictor(Constants.kRightDriveSlaveBId,
                 Constants.kRightDriveMasterId);
         //rightSlaveB_.setInverted(true);
 
         mPigeonIMU = new PigeonIMU(leftMaster_);
        //It was a slave motor in 254's code

        telep_drive = new DifferentialDrive(new SpeedControllerGroup(leftMaster_, leftSlaveA_,leftSlaveB_), 
                                        new SpeedControllerGroup(rightMaster_, rightSlaveA_,rightSlaveB_));

         
         gearShifter_ = new Solenoid(Constants.kGearShifter);
         frontLifter_ = new Solenoid(Constants.kFrontLifter);
         backLifter_ = new Solenoid(Constants.kBackLifter);

         isHighGear_ = false;
         isBackLifted_ = false;
         isFrontLifted_ = false;
         setGains();
 
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
        setHeading(Rotation2d.default_);
        resetEncoders();
        autoShift_ = true;

        gearShifter_.set(false);
        frontLifter_.set(false);
        backLifter_.set(false);
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegreeFromCoord());

        //mGyroOffset = heading.rotateFromAnother(Rotation2d.fromAngle(mPigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegreeFromCoord());

        feedData_.gyro_heading = heading;
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentDriveState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            autoShift_ = true;
            /*
            System.out.println("Switching to open loop");
            System.out.println(signal);
            currentDriveState = DriveControlState.OPEN_LOOP;
            leftMaster_.configNeutralDeadband(0.04, 0);
            rightMaster_.configNeutralDeadband(0.04, 0);*/
        }
        //feedData_.left_demand = signal.getLeft();
        //feedData_.right_demand = signal.getRight();
        feedData_.xspeed=signal.getSpeed();
        feedData_.zrotation=signal.getRotation();

        feedData_.left_feedforward = 0.0;
        feedData_.right_feedforward = 0.0;
    }

    public synchronized Rotation2d getHeading() {
        return feedData_.gyro_heading;
    }

    /**
     * I don't know where it will be used
     */
    public void registerEnabledLoops(ICycle_in regist) {
        regist.addSubsystem(cycle_);
    }

    /**
     * this is the autonomous feeding method
     * @param signal
     * @param feedforward
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (currentDriveState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            setBrakeMode(true);
            autoShift_ = false;
            
            leftMaster_.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            rightMaster_.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            leftMaster_.configNeutralDeadband(0.0, 0);
            rightMaster_.configNeutralDeadband(0.0, 0);

            currentDriveState = DriveControlState.OPEN_LOOP;// Wrong Code
        }
        feedData_.left_demand = signal.getLeft();
        feedData_.right_demand = signal.getRight();
        feedData_.left_feedforward = feedforward.getLeft();
        feedData_.right_feedforward = feedforward.getRight();
    }

    public synchronized void setGains() {
        /*
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
        */
    }

    public enum DriveControlState {
        OPEN_LOOP, // percent output control
        PATH_FOLLOWING, // motion profile control
    }

    public static class FeedData {
        // Motion Profile Feedback
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.default_;
        public Pose2d error = Pose2d.default_;

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
        //public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
    public enum ShifterState {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR,
        AUTO_SHIFT
    }

    /**
     * override from iSubsystem
     * move the drive base depends the state(telep or autonomous)
     */
    @Override
    public synchronized void move_subsystem(){
        //System.out.println(currentDriveState);
        if (currentDriveState == DriveControlState.OPEN_LOOP) {
            /*
            leftMaster_.set(ControlMode.PercentOutput, feedData_.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            rightMaster_.set(ControlMode.PercentOutput, feedData_.right_demand, DemandType.ArbitraryFeedForward, 0.0);
            */
            telep_drive.arcadeDrive(feedData_.xspeed, feedData_.zrotation);
            //System.out.println(feedData_.xspeed + " " + feedData_.zrotation);
        } else {
            leftMaster_.set(ControlMode.Velocity, feedData_.left_demand, DemandType.ArbitraryFeedForward,
                    feedData_.left_feedforward + Constants.kDriveLowGearVelocityKd * feedData_.left_accel / 1023.0);
            rightMaster_.set(ControlMode.Velocity, feedData_.right_demand, DemandType.ArbitraryFeedForward,
                    feedData_.right_feedforward + Constants.kDriveLowGearVelocityKd * feedData_.right_accel / 1023.0);
        }
    }

    @Override
    public synchronized void update_subsystem(){
        /*
            double prevLeftTicks = feedData_.left_position_ticks;
            double prevRightTicks = feedData_.right_position_ticks;
            feedData_.left_position_ticks = leftMaster_.getSelectedSensorPosition(0);
            feedData_.right_position_ticks = rightMaster_.getSelectedSensorPosition(0);
            feedData_.left_velocity_ticks_per_100ms = leftMaster_.getSelectedSensorVelocity(0);
            feedData_.right_velocity_ticks_per_100ms = rightMaster_.getSelectedSensorVelocity(0);
            //feedData_.gyro_heading = Rotation2d.fromAngle(mPigeonIMU.getFusedHeading()).rotateFromAnother(mGyroOffset);
    
            double deltaLeftTicks = ((feedData_.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
            if (deltaLeftTicks > 0.0) {
                feedData_.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            } else {
                feedData_.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            }
    
            double deltaRightTicks = ((feedData_.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
            if (deltaRightTicks > 0.0) {
                feedData_.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            } else {
                feedData_.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            }*/
    
            /* // no idea what that is
            if (mCSVWriter != null) {
                mCSVWriter.add(feedData_);
            }*/
    
            // System.out.println("control state: " + mDriveControlState + ", left: " + feedData_.left_demand + ", right: " + feedData_.right_demand);
    }

    @Override
    public boolean checkSubsystem() {
        return true;
    }
}