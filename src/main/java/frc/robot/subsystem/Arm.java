package frc.robot.subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import frc.robot.Constants;
import static frc.robot.Constants.SuperStructureConstants;
import frc.lib.motor.MotorUtility;
import frc.lib.utility.LatchBoolean;

public class Arm extends SingleMasterMotorSystem{
    private static Arm instance_;

    private boolean isBackingToInitialMode_ = false;
    private boolean canBackToInitalMode_ = true;
    private LatchBoolean changeTester = new LatchBoolean();
    private int level1;
    private int vertex;//the point where wrist 

    public synchronized static Arm getInstance() {
        if (instance_ == null) {
            instance_ = new Arm(Constants.kArm);
        }

        return instance_;
    }

    public Arm(final Constants.SingleMasterMotorSystemConfig constant){
        super(constant);

        int startMatch = master_.getSelectedSensorPosition();
        level1 = startMatch + SuperStructureConstants.startmatch_from_level1;

        vertex = level1 + SuperStructureConstants.vertex_from_lvl1;
        int reverselimit = level1 + SuperStructureConstants.reverselimit_from_lvl1;
        int forwardlimit = level1 + SuperStructureConstants.forwardlimit_from_lvl1;

        MotorUtility.checkError(master_.configReverseSoftLimitThreshold(reverselimit),
            "Unable to configReverseSoftLimitThreshold(reverselimit) for arm");

        MotorUtility.checkError(master_.configForwardSoftLimitThreshold(forwardlimit),
            "Unable to configForwardSoftLimitThreshold(forwardlimit) for arm");

        MotorUtility.checkError(master_.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen, Constants.kLongCANTimeoutMs), "Unable to set reverse limit switch for arm.");

        master_.overrideLimitSwitchesEnable(true);
    }

    public int getLevelOne() {
        return level1;
    }

    public int getVextex(){
        return vertex;
    }

    public synchronized void setCanBackToInitalMode(boolean canBack) {
        canBackToInitalMode_ = canBack;
    }

    /**
     * getBoolean if the the reverse limit switch is closed to determin whether
     * the arm is at inital mode
     * @return the result
     */
    public synchronized boolean isAtInitialMode() {
        return master_.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (changeTester.getBoolean(reset) && canBackToInitalMode_) {
            System.out.println("Arm going into home mode!");
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

                master_.overrideSoftLimitsEnable(true);
                isBackingToInitialMode_ = false;
            }

            if(controlMode_ == ControlType.OPEN_LOOP) {
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
    
    @Override 
    public void sendDataToSmartDashboard(){
        SmartDashboard.putNumber("Arm Value", master_.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm Error",master_.getClosedLoopError());
        SmartDashboard.putNumber("Arm Current:", master_.getOutputCurrent());
    }

    @Override
    public boolean checkSubsystem(){
        return true;
    }

}
