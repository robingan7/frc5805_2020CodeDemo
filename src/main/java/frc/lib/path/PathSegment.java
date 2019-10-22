package frc.lib.path;

import frc.lib.waypoint.*;

public class PathSegment{
    private Translation2d start;
    private Translation2d end;
    private Translation2d center;
    private Translation2d deltaStart;
    private Translation2d deltaEnd;
    private double maxSpeed;
    private boolean isLine;
    //private MotionProfile speedController;
    private boolean extrapolateLookahead;
    private String marker;
}