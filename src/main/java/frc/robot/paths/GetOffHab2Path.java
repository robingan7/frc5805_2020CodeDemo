package frc.robot.paths;

import frc.lib.path.PathBuilder.Waypoint;
import frc.lib.path.Path;
import frc.lib.waypoint.Pose2D;
import frc.lib.waypoint.Rotation2D;
import frc.lib.waypoint.Translation2D;
import frc.lib.path.PathHolder;
import frc.lib.path.PathBuilder;

import java.util.ArrayList;

public class GetOffHab2Path implements PathHolder {

    @Override
    public Path makePath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(25, 0, 0, 20));
        //sWaypoints.add(new Waypoint(100, 0, 0.0, 100.0));
        //sWaypoints.add(new Waypoint(100, 100, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    public Pose2D getStartPose() {
        return new Pose2D(new Translation2D(0, 0), Rotation2D.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}