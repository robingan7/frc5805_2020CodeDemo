package frc.lib.path;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 * @see 
 */
public interface PathHolder{
    Path makePath();

    boolean isReversed();
}
