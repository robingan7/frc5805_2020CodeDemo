package frc.robot.action;

import frc.robot.subsystem.Drivebase;

public class ForceEndPathAction extends RunOnceAction {

    @Override
    public synchronized void runOnce() {
        Drivebase.getInstance().forceDoneWithPath();
    }
}