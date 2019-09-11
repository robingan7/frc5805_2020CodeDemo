package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.lib.motor.MotorUtil;
import frc.lib.utility.TestIfChanged;

public class Arm extends SuperStructureSubsystem{
    private static Arm instance_;

    private boolean isBackingToInitialMode_ = false;
    private boolean canBackToInitalMode_ = true;
    private TestIfChanged changeTester = new TestIfChanged();

    public synchronized static Arm getInstance() {
        if (instance_ == null) {
            instance_ = new Arm(Constants.kArm);
        }

        return instance_;
    }

    public Arm(final Constants.SuperStructureSubsystemConstants constant){
        super(constant);

        MotorUtil.checkError(master_.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                LimitSwitchNormal.NormallyOpen, Constants.kLongCANTimeoutMs),
                                "Unable to set reverse limit switch for elevator.");
        master_.overrideLimitSwitchesEnable(true);                        
    }


    public synchronized void setCanBackToInitalMode(boolean canBack) {
        canBackToInitalMode_ = canBack;
    }

    /**
     * test if the the reverse limit switch is closed to determin whether
     * the arm is at inital mode
     * @return the result
     */
    public synchronized boolean isAtInitialMode() {
        return master_.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (changeTester.test(reset) && canBackToInitalMode_) {
            System.out.println("Elevator going into home mode!");
            isBackingToInitialMode_ = true;
            //LED.getInstance().setElevatorFault();

            //Can be used to override-disable the soft limits. This function can be used to quickly disable soft limits without having to modify the persistent configuration.
            master_.overrideSoftLimitsEnable(false);
        }
    }

    @Override
    public synchronized void move_subsystem() {
        // special case for backing to intial mode
        if(isBackingToInitialMode_) {
            if(isAtInitialMode()){
                zeroSensors();

                //Can be used to override-disable the soft limits. This function can be used to quickly disable soft limits without having to modify the persistent configuration.
                master_.overrideSoftLimitsEnable(true);
                isBackingToInitialMode_ = false;
            }

            if(controlMode_ == SuperStructureMode.OPEN_LOOP) {
                master_.set(ControlMode.PercentOutput, feedData_.demand, DemandType.ArbitraryFeedForward,
                0.0);
            } else {
                master_.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward,
                0.0);
            }
        } else {
            super.move_subsystem();//perform most tasks
        }
    }

    public synchronized boolean getIsAtInitalMode() {
        return isBackingToInitialMode_;
    }

    public synchronized void updateSoftLimit(int limit) {
        master_.configForwardSoftLimitThreshold(limit);
    }

    public synchronized void removeCurrentLimits() {
        master_.enableCurrentLimit(false);
    }


}
