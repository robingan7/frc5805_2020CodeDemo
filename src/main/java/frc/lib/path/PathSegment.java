package frc.lib.path;

import frc.lib.waypoint.*;

public class PathSegment{
    private Translation2D start;
    private Translation2D end;
    private Translation2D center;
    private Translation2D deltaStart;
    private Translation2D deltaEnd;
    private double maxSpeed;
    private boolean isLine;
    //private MotionProfile speedController;
    private boolean extrapolateLookahead;
    private String marker;
}