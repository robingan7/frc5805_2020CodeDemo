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
    private static SuperStructureSubsystemContainer instance_;
    private final Arm arm_ = Arm.getInstance();
    private final Wrist wrist_ = Wrist.getInstance();

    private SuperStructureState currentState_ = new SuperStructureState();
    private SuperStructureGoal goal_ = null;
    private SuperStructureGoal lastValidGoal_;

    private boolean disableArmAndWrist_ = false;

    private boolean isMovingToGoal = false;

    public enum ArmControlMode {
        OPEN_LOOP, PID, MOTION_PROFILE
    }
    private ArmControlMode armControlMode_ = ArmControlMode.OPEN_LOOP;

    public static SuperStructureSubsystemContainer getInstance(){
        if (instance_ == null) {
            instance_ = new SuperStructureSubsystemContainer();
        }

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
                updateGoal();

                if (goal_ != null && isMovingToGoal) {
                    moveToGoal(); // if at desired state, this should stabilize the superstructure at that state
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
    
    private void updateGoal(){
        if (goal_ != null) {

            if (lastValidGoal_ == null) {
                lastValidGoal_ = new SuperStructureGoal(goal_.state_);
            }

            if (goal_.isAtDesiredState(currentState_) && !goal_.equals(lastValidGoal_)) {
                lastValidGoal_.state_.setState(goal_.state_);
                isMovingToGoal = false;
                armControlMode_ = ArmControlMode.OPEN_LOOP;
            } else {
                isMovingToGoal = true;
                armControlMode_ = ArmControlMode.PID;
            }
        }
    }

    private void updateCurrentState(){
        currentState_.arm_ = arm_.getPosition();
        currentState_.wrist_ = wrist_.getPosition();
    }

    private synchronized void moveToGoal() {
        if (disableArmAndWrist_) {
            arm_.setOpenLoop(0.0);
            wrist_.setOpenLoop(0.0);
        } else {
            arm_.setSetpointMotionMagic(goal_.state_.arm_);
            wrist_.setSetpointMotionMagic(goal_.state_.wrist_);
            //System.out.println("Goal" + goal_.state_.wrist_ + " / " + goal_.state_.arm_);
        }
    }

    @Override
    public boolean checkSubsystem(){
        return false;
    }

    public synchronized boolean isAtDesiredState() {
        return goal_ == null || (goal_.isAtDesiredState(currentState_) && !goal_.equals(lastValidGoal_));
    }

    public synchronized void setGoal(SuperStructureGoal goal, ArmControlMode armControlMode) {
        if (goal_ == null) {
            goal_ = new SuperStructureGoal(goal.state_);
        }

        armControlMode_ = armControlMode;
        
        if (armControlMode_ != ArmControlMode.OPEN_LOOP) {
            goal_.state_.arm_ = goal.state_.arm_;
        }
        goal_.state_.wrist_ = goal.state_.wrist_;
    }

    public synchronized void setGoal(SuperStructureGoal goal) {
        setGoal(goal, ArmControlMode.PID);
    }

    /**
     * this method is used when our wrist or arm is off positioned by other robot. 
     */
    public synchronized void moveToLastGoal() {
        setGoal(lastValidGoal_);
    } 

    public synchronized SuperStructureGoal getGoal() {
        return goal_;
    }
} 