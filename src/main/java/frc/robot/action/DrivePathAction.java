package frc.robot.action;

import frc.lib.path.PathHolder;
import frc.robot.subsystem.Drivebase;
import frc.lib.path.Path;
import frc.lib.utility.DriveSignal;

/**
 * Drives the robot along the Path defined in the PathHolder object. The action finishes once the robot reaches the
 * end of the path.
 *
 * @see PathHolder
 * @see Path
 * @see Action
 */
public class DrivePathAction implements Action {

    private PathHolder mPathContainer;
    private Path mPath;
    private Drivebase drive_ = Drivebase.getInstance();
    private boolean mStopWhenDone;

    public DrivePathAction(PathHolder p, boolean stopWhenDone) {
        mPathContainer = p;
        mPath = mPathContainer.makePath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathHolder p) {
        this(p, false);
    }

    @Override
    public void start() {
        drive_.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return drive_.isDoneWithPath();
    }

    @Override
    public void done() {
        if (mStopWhenDone) {
            drive_.setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }
}