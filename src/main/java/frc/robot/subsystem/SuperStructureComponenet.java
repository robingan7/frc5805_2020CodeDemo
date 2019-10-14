package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.cycle.*;
import frc.lib.motor.MotorUtil;

public abstract class SuperStructureComponenet extends Subsystem_Function {
    private static final int kPositionPIDSlot = 0;

    protected final Constants.SuperStructurComponentConstants constants_;
    protected final WPI_TalonSRX master_;
    protected final WPI_VictorSPX[] slaves_;
   
    protected SuperStructureComponenet(final Constants.SuperStructurComponentConstants constants){
        constants_ = constants;
        master_ = MotorUtil.createTalon(constants_.kMasterConstants.id);
        slaves_ = new WPI_VictorSPX[constants_.kSlaveConstants.length];
        master_.enableCurrentLimit(true);

        master_.configVoltageMeasurementFilter(8);

        MotorUtil.checkError(master_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,
                Constants.kLongCANTimeoutMs), constants_.kName + ": Could not detect encoder: ");

        MotorUtil.checkError(master_.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not enable forward soft limit: ");

        MotorUtil.checkError(master_.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not enable reverse soft limit: ");

        MotorUtil.checkError(master_.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set voltage compensation saturation: ");

        MotorUtil.checkError(master_.config_kP(Constants.kSlotIdx, constants_.kKp, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kP: ");

        MotorUtil.checkError(master_.config_kI(Constants.kSlotIdx, constants_.kKi, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kI: ");

        MotorUtil.checkError(master_.config_kD(Constants.kSlotIdx, constants_.kKd, Constants.kLongCANTimeoutMs),
                constants_.kName + ": could not set kD: ");

        MotorUtil.checkError(master_.config_kF(Constants.kSlotIdx, constants_.kKf, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set kF: ");

        MotorUtil.checkError(master_.configMaxIntegralAccumulator(Constants.kSlotIdx, constants_.kMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs), constants_.kName + ": Could not set max integral: ");

        MotorUtil.checkError(
                master_.configAllowableClosedloopError(Constants.kSlotIdx, constants_.kDeadband, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set deadband: ");

        MotorUtil.checkError(master_.configMaxIntegralAccumulator(kPositionPIDSlot, constants_.kPositionMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs), constants_.kName + ": Could not set max integral: ");

        MotorUtil.checkError(master_.config_IntegralZone(kPositionPIDSlot, constants_.kPositionIZone, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set i zone: ");

        MotorUtil.checkError(
                master_.configAllowableClosedloopError(kPositionPIDSlot, constants_.kPositionDeadband, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set deadband: ");

        MotorUtil.checkError(
                master_.configMotionCruiseVelocity(constants_.kCruiseVelocity, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set cruise velocity: ");

        MotorUtil.checkError(master_.configMotionAcceleration(constants_.kAcceleration, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set acceleration: ");

        MotorUtil.checkError(master_.configOpenloopRamp(constants_.kRampRate, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set voltage ramp rate: ");

        MotorUtil.checkError(master_.configClosedloopRamp(constants_.kRampRate, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set closed loop ramp rate: ");

        MotorUtil.checkError(
                master_.configContinuousCurrentLimit(constants_.kContinuousCurrentLimit, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set continuous current limit.");

        MotorUtil.checkError(
                master_.configPeakCurrentLimit(constants_.kPeakCurrentLimit, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set peak current limit.");

        MotorUtil.checkError(
                master_.configPeakCurrentDuration(constants_.kPeakCurrentDuration, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set peak current duration.");
        master_.enableCurrentLimit(true);

        master_.configVoltageMeasurementFilter(8);

        master_.configNominalOutputForward(constants_.kNominalOutputForward, Constants.kLongCANTimeoutMs);
        master_.configNominalOutputReverse(constants_.kNominalOutputReverse, Constants.kLongCANTimeoutMs);

        MotorUtil.checkError(
                master_.configVoltageCompSaturation(constants_.kMaxVoltage, Constants.kLongCANTimeoutMs),
                constants_.kName + ": Could not set voltage comp saturation.");
        master_.enableVoltageCompensation(true);

        master_.setInverted(constants_.kMasterConstants.invert_motor);
        master_.setSensorPhase(constants_.kMasterConstants.invert_sensor_phase);
        master_.setNeutralMode(NeutralMode.Brake);
        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        master_.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, constants_.kStastusFrame8UpdateRate, 20);

        // Start with Constants.kSlotIdx.
        master_.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);

        for (int i = 0; i < slaves_.length; ++i) {
            slaves_[i] = MotorUtil.createVictor(constants_.kSlaveConstants[i].id,
                    constants_.kMasterConstants.id);
            slaves_[i].setInverted(constants_.kSlaveConstants[i].invert_motor);
            slaves_[i].setNeutralMode(NeutralMode.Brake);
            slaves_[i].follow(master_);
        }

        stop();
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public static class FeedData{
        // INPUTS
        public double timestamp;
        public int position_ticks;
        public double position_units;
        public double output_percent;
        public double output_voltage;
        public double master_current;
        public boolean wantReset;

        // OUTPUTS
        public double demand;
        public double feedforward;
    }

    protected enum SuperStructureMode {
        OPEN_LOOP, MOTION_MAGIC, POSITION_PID, MOTION_PROFILING
    }

    protected FeedData feedData_ = new FeedData();
    protected SuperStructureMode controlMode_ = SuperStructureMode.OPEN_LOOP;
    protected final Cycle registCycle_ = new Cycle() {
        @Override
        public void onStart(double timestamp) {
            // if (mCSVWriter == null) {
            //     mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/"
            //             + constants_.kName.replaceAll("[^A-Za-z0-9]+", "").toUpperCase() + "-LOGS.csv",
            //             PeriodicIO.class);
            // }
        }

        @Override
        public void onLoop(double timestamp) {
            if (feedData_.wantReset) {
                System.out.println(constants_.kName + ": Master Talon reset occurred; resetting frame rates.");
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
                master_.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, constants_.kStastusFrame8UpdateRate, 20);

                // Reset encoder position to estimated position from absolute encoder
                if (constants_.kRecoverPositionOnReset) {
                    //master_.setSelectedSensorPosition(estimateSensorPositionFromAbsolute(), 0, Constants.kCANTimeoutMs);
                }
            }
            handleMasterReset(feedData_.wantReset);
            for (WPI_VictorSPX slave : slaves_) {
                if (slave.hasResetOccurred()) {
                    System.out.println(constants_.kName + ": Slave Talon reset occurred");
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            /*
            if (mCSVWriter != null) {
                mCSVWriter.flush();
                mCSVWriter = null;
            }*/

            stop();
        }
    };

    /**
     * call if the subsystem want to be reset, and should be override
     * @param reset the feeData.wantReset
     */
    public synchronized void handleMasterReset(boolean reset) {}

    @Override
    public synchronized void move_subsystem(){
        if (controlMode_ == SuperStructureMode.MOTION_MAGIC) {
            master_.set(ControlMode.MotionMagic, feedData_.demand, DemandType.ArbitraryFeedForward,
                         feedData_.feedforward);
        } else if (controlMode_ == SuperStructureMode.POSITION_PID || controlMode_ == SuperStructureMode.MOTION_PROFILING) {
            master_.set(ControlMode.Position, feedData_.demand, DemandType.ArbitraryFeedForward,
                    feedData_.feedforward);
        } else {
            master_.set(ControlMode.PercentOutput, feedData_.demand, DemandType.ArbitraryFeedForward,
                        feedData_.feedforward);
        }
        
    }

    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        //feedData_.demand = constrainTicks(homeAwareUnitsToTicks(units));
        feedData_.feedforward = units;
        if (controlMode_ != SuperStructureMode.MOTION_MAGIC) {
            master_.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
            controlMode_ = SuperStructureMode.MOTION_MAGIC;
        }
    }

    public synchronized void setSetpointMotionMagic(double units) {
        setSetpointMotionMagic(units, 0.0);
    }

    /**
     * start percentage control
     */
    public synchronized void setOpenLoop(double percentage) {
        feedData_.demand = percentage;
        if (controlMode_ != SuperStructureMode.OPEN_LOOP) {
            controlMode_ = SuperStructureMode.OPEN_LOOP;
        }
    }

    @Override
    public void registerEnabledLoops(ICycle_in enabledLooper) {
        enabledLooper.addSubsystem(registCycle_);
    }

    public synchronized double getPosition() {
        return ticksToHomedUnits(feedData_.position_ticks);
    }

    //utility function 
    protected double ticksToUnits(double ticks) {
        return ticks / constants_.kTicksPerUnitDistance;
    }

    protected double ticksToHomedUnits(double ticks) {
        double val = ticksToUnits(ticks);
        return val + constants_.kHomePosition;
    }
}