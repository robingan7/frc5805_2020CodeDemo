package frc.robot.action;

import frc.robot.subsystem.SuperStructureSubsystemContainer;

public abstract class SuperstructureActionBase implements Action{
    protected final SuperStructureSubsystemContainer superStructure_ = SuperStructureSubsystemContainer.getInstance();
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