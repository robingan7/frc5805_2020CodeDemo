package frc.robot.paths;

import frc.lib.path.Path;
import frc.lib.path.PathHolder;
import frc.lib.path.PathBuilder;

import java.util.ArrayList;

public class Hab1ToCargoShip1Path implements PathHolder {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";

    boolean mLeft;

    public Hab1ToCargoShip1Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path makePath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(0, 0.0, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(30, 0.0, 0, 40.0, kStartAutoAimingMarker));
        sWaypoints.add(new PathBuilder.Waypoint(110, (mLeft ? 1.0 : -1.0) * 1.5, 1, 100.0, kStartAutoAimingMarker));
        sWaypoints.add(new PathBuilder.Waypoint(205, (mLeft ? 1.0 : -1.0) * 3, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}