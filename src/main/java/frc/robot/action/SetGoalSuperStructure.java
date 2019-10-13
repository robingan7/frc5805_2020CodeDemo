package frc.robot.action;

import frc.robot.statesAndMachanics.SuperStructureState;
import frc.robot.statesAndMachanics.SuperStructureGoal;

public class SetGoalSuperStructure extends SuperstructureActionBase{
    private SuperStructureState forwardState_;

    public SetGoalSuperStructure(SuperStructureState forwardState){
        forwardState_ = forwardState;
    }

    @Override
    public void start(){
        //superStructure_.setGoal(new SuperStructureGoal(forwardState_));
    }
}