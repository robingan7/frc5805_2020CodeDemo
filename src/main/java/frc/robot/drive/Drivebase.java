package frc.robot.drive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.lib.waypoint.*;

import frc.lib.utility.DriveSignal;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import frc.robot.cycle.*;
import frc.lib.motor.MotorUtil;
import frc.robot.Constants;
import frc.robot.subsystem.Subsystem_Function;

public class Drivebase extends Subsystem_Function{
    private final WPI_TalonSRX mLeftMaster, mRightMaster;
    private final WPI_VictorSPX mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB;

    private static Drivebase instance_;
    private FeedData mFeedData;
    private PigeonIMU mPigeonIMU;
    private DifferentialDrive telep_drive;

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;

    private Rotation2d mGyroOffset = Rotation2d.default_;

    private DriveControlState currentDriveState;
    private boolean mIsBrakeMode,mIsHighGear,mAutoShift;

    private Solenoid mShifter;
    private final Cycle mCycle = new Cycle() {
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
                {
                    setHighGear(false);
                }
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
        mFeedData=new FeedData();
         // Start all Talons in open loop mode.
         mLeftMaster = MotorUtil.createTalon( Constants.kLeftDriveMasterId);
         configureMaster(mLeftMaster, true);
 
         mLeftSlaveA = MotorUtil.createVictor(Constants.kLeftDriveSlaveAId,
                 Constants.kLeftDriveMasterId);
         mLeftSlaveA.setInverted(false);
 
         mLeftSlaveB = MotorUtil.createVictor(Constants.kLeftDriveSlaveBId,
                 Constants.kLeftDriveMasterId);
         mLeftSlaveB.setInverted(false);

 
         mRightMaster = MotorUtil.createTalon(Constants.kRightDriveMasterId);
         configureMaster(mRightMaster, true);
 
         mRightSlaveA = MotorUtil.createVictor(Constants.kRightDriveSlaveAId,
                 Constants.kRightDriveMasterId);
         //mRightSlaveA.setInverted(true);
 
         mRightSlaveB = MotorUtil.createVictor(Constants.kRightDriveSlaveBId,
                 Constants.kRightDriveMasterId);
         //mRightSlaveB.setInverted(true);
 
         mPigeonIMU=new PigeonIMU(mLeftMaster);
        //It was a slave motor in 254's code

        telep_drive=new DifferentialDrive(new SpeedControllerGroup(mLeftMaster, mLeftSlaveA,mLeftSlaveB), 
                                        new SpeedControllerGroup(mRightMaster, mRightSlaveA,mRightSlaveB));

         
         mShifter = MotorUtil.makeSolenoidFromId(Constants.kShifterSolenoidId);

         setGains();
 
    }
    
    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(wantsHighGear);
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
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mFeedData = new FeedData();
    }

    public void zeroSensors() {
        setHeading(Rotation2d.default_);
        resetEncoders();
        mAutoShift = true;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegreeFromCoord());

        //mGyroOffset = heading.rotateFromAnother(Rotation2d.fromAngle(mPigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegreeFromCoord());

        mFeedData.gyro_heading = heading;
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (currentDriveState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(false);
            mAutoShift = true;
            /*
            System.out.println("Switching to open loop");
            System.out.println(signal);
            currentDriveState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);*/
        }
        //mFeedData.left_demand = signal.getLeft();
        //mFeedData.right_demand = signal.getRight();
        mFeedData.xspeed=signal.getSpeed();
        mFeedData.zrotation=signal.getRotation();

        mFeedData.left_feedforward = 0.0;
        mFeedData.right_feedforward = 0.0;
    }

    public synchronized Rotation2d getHeading() {
        return mFeedData.gyro_heading;
    }

    /**
     * I don't know where it will be used
     */
    public void registerEnabledLoops(ICycle_in regist) {
        regist.addSubsystem(mCycle);
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
            mAutoShift = false;
            /*
            mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);*/

            currentDriveState = DriveControlState.OPEN_LOOP;// Wrong Code
        }
        mFeedData.left_demand = signal.getLeft();
        mFeedData.right_demand = signal.getRight();
        mFeedData.left_feedforward = feedforward.getLeft();
        mFeedData.right_feedforward = feedforward.getRight();
    }

    public synchronized void setGains() {
        /*
        mLeftMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
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
        System.out.println(currentDriveState);
        if (currentDriveState == DriveControlState.OPEN_LOOP) {
            /*
            mLeftMaster.set(ControlMode.PercentOutput, mFeedData.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mFeedData.right_demand, DemandType.ArbitraryFeedForward, 0.0);
            */
            telep_drive.arcadeDrive(mFeedData.xspeed, mFeedData.zrotation);
            System.out.println(mFeedData.xspeed + " " + mFeedData.zrotation);
        } else {
            mLeftMaster.set(ControlMode.Velocity, mFeedData.left_demand, DemandType.ArbitraryFeedForward,
                    mFeedData.left_feedforward + Constants.kDriveLowGearVelocityKd * mFeedData.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mFeedData.right_demand, DemandType.ArbitraryFeedForward,
                    mFeedData.right_feedforward + Constants.kDriveLowGearVelocityKd * mFeedData.right_accel / 1023.0);
        }
    }

    @Override
    public synchronized void update_subsystem(){
        /*
            double prevLeftTicks = mFeedData.left_position_ticks;
            double prevRightTicks = mFeedData.right_position_ticks;
            mFeedData.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
            mFeedData.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
            mFeedData.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
            mFeedData.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
            //mFeedData.gyro_heading = Rotation2d.fromAngle(mPigeonIMU.getFusedHeading()).rotateFromAnother(mGyroOffset);
    
            double deltaLeftTicks = ((mFeedData.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
            if (deltaLeftTicks > 0.0) {
                mFeedData.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            } else {
                mFeedData.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            }
    
            double deltaRightTicks = ((mFeedData.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
            if (deltaRightTicks > 0.0) {
                mFeedData.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            } else {
                mFeedData.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            }*/
    
            /* // no idea what that is
            if (mCSVWriter != null) {
                mCSVWriter.add(mFeedData);
            }*/
    
            // System.out.println("control state: " + mDriveControlState + ", left: " + mFeedData.left_demand + ", right: " + mFeedData.right_demand);
    }
}