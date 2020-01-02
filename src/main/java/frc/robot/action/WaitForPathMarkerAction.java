package frc.robot.action;

import frc.robot.subsystem.Drivebase;

public class WaitForPathMarkerAction implements Action {

    private Drivebase mDrive = Drivebase.getInstance();
    private String mMarker;

    public WaitForPathMarkerAction(String marker) {
        mMarker = marker;
    }

    @Override
    public boolean isFinished() {
        return mDrive.hasPassedMarker(mMarker);
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}