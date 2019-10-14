package frc.robot.subsystem;

import frc.robot.cycle.Cycle;
import frc.robot.cycle.ICycle_in;
import frc.robot.statesAndMachanics.SuperStructureState;
import frc.robot.statesAndMachanics.SuperStructureGoal;

/**
 * this class container all the superstucture(arm, and wrist). 
 * It contains primary functions that move the motors
 */

public class SuperStructureSubsystemContainer extends Subsystem_Function{
    private final static SuperStructureSubsystemContainer instance_ = new SuperStructureSubsystemContainer();
    private final Arm arm_ = Arm.getInstance();
    private final Wrist wrist_ = Wrist.getInstance();

    private SuperStructureState currentState_ = new SuperStructureState();
    private SuperStructureGoal currentSetPoint_ = null;
    private SuperStructureGoal goal_;
    private SuperStructureGoal preWristLevelGoal_;
    private SuperStructureGoal lastValidGoal_;

    private boolean disableArmAndWrist_ = false;

    public enum ArmControlMode {
        OPEN_LOOP, PID, PRISMATIC_WRIST
    }
    private ArmControlMode armControlMode_ = ArmControlMode.OPEN_LOOP;

    public static SuperStructureSubsystemContainer getInstance(){
        return instance_;
    }
    private SuperStructureSubsystemContainer(){}
    
    private Cycle cycle_ = new Cycle(){
        @Override
        public void onStart(double timestamp){
            synchronized(SuperStructureSubsystemContainer.this){
                armControlMode_ = ArmControlMode.OPEN_LOOP;
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized(SuperStructureSubsystemContainer.this){
                updateCurrentState();
                updateSetpointFromGoal();

                if (currentSetPoint_ != null) {
                    followSetpoint(); // if at desired state, this should stabilize the superstructure at that state
                }
            }
        }
        @Override
        public void onStop(double timestamp){
            stop();
        }
    };

    @Override
    public void registerEnabledLoops(ICycle_in registLoop){
        registLoop.addSubsystem(cycle_);
    }

    @Override
    public void stop(){}
    
    private void updateSetpointFromGoal(){
        if (currentSetPoint_ == null) {
            currentSetPoint_ = new SuperStructureGoal(currentState_);
        }

        if (lastValidGoal_ == null) {
            lastValidGoal_ = new SuperStructureGoal(currentState_);
        }

        if (currentSetPoint_.isAtDesiredState(currentState_) || !goal_.equals(lastValidGoal_)) {
            currentSetPoint_ = goal_;
        }
        lastValidGoal_.state_.setState(goal_.state_);
    }

    private void updateCurrentState(){
        currentState_.arm_ = arm_.getAngle();
        currentState_.wrist_ = wrist_.getAngle();
    }

    private synchronized void followSetpoint() {
        if (disableArmAndWrist_) {
            arm_.setOpenLoop(0.0);
            wrist_.setOpenLoop(0.0);
        } else {
            arm_.setSetpointMotionMagic(currentSetPoint_.state_.arm_);
            wrist_.setSetpointMotionMagic(currentSetPoint_.state_.wrist_);
        }
    }

    @Override
    public boolean checkSubsystem(){
        return false;
    }

    public synchronized boolean isAtDesiredState() {
        return currentState_ != null && goal_ != null && goal_.isAtDesiredState(currentState_);
    }

    public synchronized void setGoal(SuperStructureGoal goal, ArmControlMode armControlMode) {
        if (goal_ == null) {
            goal_ = new SuperStructureGoal(goal.state_);
        }

        if (armControlMode_ == ArmControlMode.PRISMATIC_WRIST && armControlMode_ != armControlMode) {
                preWristLevelGoal_ = new SuperStructureGoal(goal_.state_);
        }

        if (armControlMode_ != ArmControlMode.PRISMATIC_WRIST) {
                preWristLevelGoal_ = null;
        }

        armControlMode_ = armControlMode;
        
        if (armControlMode_ != ArmControlMode.PRISMATIC_WRIST) {
            goal_.state_.arm_ = goal.state_.arm_;
        }
        goal_.state_.wrist_ = goal.state_.wrist_;
    }

    public synchronized void setGoal(SuperStructureGoal goal) {
        setGoal(goal, ArmControlMode.PID);
    }

    public synchronized SuperStructureGoal getPreWristLevelGoal() {
        return preWristLevelGoal_;
    }

    public synchronized void overridePreWristLevelGoal(SuperStructureGoal goal) {
        preWristLevelGoal_ = goal;
    }

    public synchronized SuperStructureGoal getGoal() {
        return goal_;
    }

} 