package frc.robot.statesAndMachanics;

import static frc.robot.Constants.SuperStructureConstants;

public class SuperStructureGoal{
    public final SuperStructureState state_;

    public SuperStructureGoal(double arm, double wrist){
        this(new SuperStructureState(arm, wrist));
    }

    public SuperStructureGoal(SuperStructureState state){
        state_ = new SuperStructureState(state);
    }

    public boolean isEquals(SuperStructureGoal other){
        return state_.arm_ == other.state_.arm_ &&
                state_.wrist_ == other.state_.wrist_;
    }

    public boolean isAtDesiredState(SuperStructureState currentState) {
        double[] distances = {
                currentState.arm_ - state_.arm_,
                currentState.wrist_ - state_.wrist_
        };

        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperStructureConstants.kPadding[i]) {
                return false;
            }
        }

        return true;
    }
}