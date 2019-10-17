
package frc.lib.utility;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double leftMotor_;
    protected double rightMotor_;
    protected boolean brakeMode_;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        leftMotor_ = left;
        rightMotor_ = right;
        brakeMode_ = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return leftMotor_;
    }

    public double getRight() {
        return rightMotor_;
    }


    public double getSpeed() {
        return leftMotor_;
    }

    public double getRotation() {
        return rightMotor_;
    }

    public boolean getBrakeMode() {
        return brakeMode_;
    }

    @Override
    public String toString() {
        return "L: " + leftMotor_ + ", R: " + rightMotor_ + (brakeMode_ ? ", BRAKE" : "");
    }
}