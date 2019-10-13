package frc.robot.subsystem;

import frc.robot.cycle.Cycle;
import frc.robot.cycle.ICycle_in;
import frc.robot.statesAndMachanics.SuperStructureState;
import frc.robot.statesAndMachanics.SuperStructureGoal;
import edu.wpi.first.hal.communication.*;

/**
 * this class container all the superstucture(arm, and wrist). 
 * It contains primary functions that move the motors
 */

public class SuperStructureSubsystemContainer extends Subsystem_Function{
    private final static SuperStructureSubsystemContainer instance_ = new SuperStructureSubsystemContainer();
    private final Arm  arm_ = Arm.getInstance();

    private SuperStructureState currentState_ = new SuperStructureState();
    private SuperStructureGoal currentSetPoint_ = null;
    private SuperStructureGoal goal_;

    public enum ArmControlMode {
        OPEN_LOOP, PID,PRISMATIC_WRIST
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
                updateArgoal_();
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
    
    public void updateArgoal_(){}

    @Override
    public boolean checkSubsystem(){
        return false;
    }

    public synchronized boolean isAtDesiredState() {
        return  currentState_ != null && goal_ != null && goal_.isAtDesiredState(currentState_);
    }

    public synchronized void setGoal(SuperStructureGoal goal, ArmControlMode armControlMode) {
        if (goal_ == null) {
            goal_ = new SuperStructureGoal(goal.state_);
        }

        if (armControlMode_ == ArmControlMode.PRISMATIC_WRIST
                && armControlMode_ != armControlMode) {
          //  mPreWristLevelGoal = new SuperStructureGoal(goal_.state_);
        }
        if (armControlMode_ != ArmControlMode.PRISMATIC_WRIST) {
           // mPreWristLevelGoal = null;
        }

        armControlMode_ = armControlMode;

        
        if (armControlMode_ != ArmControlMode.PRISMATIC_WRIST) {
            goal_.state_.arm_ = goal.state_.arm_;
        }
        goal_.state_.wrist_ = goal.state_.wrist_;
    }
} 