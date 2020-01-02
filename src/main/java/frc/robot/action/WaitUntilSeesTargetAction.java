package frc.robot.action;

import frc.robot.Constants;
import frc.robot.statesAndMachanics.RobotState;

public class WaitUntilSeesTargetAction implements Action {
    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getAimingParameters(false, -1, Constants.kMaxGoalTrackAge).isPresent(); //useHighTraget is missing
    }

    @Override
    public void done() {}
}