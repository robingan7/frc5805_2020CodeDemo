package frc.robot.action;

import frc.robot.subsystem.SuperStructure;

public abstract class SuperstructureActionBase implements Action{
    protected final SuperStructure superStructure_ = SuperStructure.getInstance();
    protected boolean waitForAction_;

    public SuperstructureActionBase(boolean waitForAction){
        waitForAction_ = waitForAction;
    }

    public SuperstructureActionBase(){
        waitForAction_ = true;
    }

    @Override
    public abstract void start();

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return !waitForAction_ || superStructure_.isAtDesiredState();
    }

    @Override
    public void done() {}
}