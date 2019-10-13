package frc.robot.subsystem;

import frc.robot.cycle.ICycle_in;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public abstract class Subsystem_Function{
    public void writeToLog() {}

    // used to move susystem
    public void move_subsystem() {}

    //update subsystem's states used in feeding profile and PID
    public void update_subsystem() {}

    public abstract boolean checkSubsystem();
    /*
    public abstract void outputTelemetry();*/

    public abstract void stop();

    public void zeroSensors() {}

    public void registerEnabledLoops(ICycle_in enabledLooper) {}
}