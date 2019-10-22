package frc.lib.path;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

/**
 * Class representing the robot's autonomous path.
 * <p>
 * Field Coordinate System: Uses a right hand coordinate system. Positive x is right, positive y is up, and the origin
 * is at the bottom left corner of the field. For angles, 0 degrees is facing right (1, 0) and angles increase as you
 * turn counter clockwise.
 */

 public class Path{
    List<PathSegment> segments;
    PathSegment prevSegment;
    HashSet<String> mMarkersCrossed = new HashSet<String>();
 }