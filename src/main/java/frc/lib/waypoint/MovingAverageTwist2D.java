package frc.lib.waypoint;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average of the Twist2D class
 */
public class MovingAverageTwist2D {
    ArrayList<Twist2D> twists = new ArrayList<Twist2D>();
    private int maxSize;

    public MovingAverageTwist2D(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Twist2D twist) {
        twists.add(twist);
        if (twists.size() > maxSize) {
            twists.remove(0);
        }
    }

    public synchronized Twist2D getAverage() {
        double x = 0.0, y = 0.0, t = 0.0;

        for (Twist2D twist : twists) {
            x += twist.dx;
            y += twist.dy;
            t += twist.dtheta;
        }

        double size = getSize();
        return new Twist2D(x / size, y / size, t / size);
    }

    public int getSize() {
        return twists.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        twists.clear();
    }

}