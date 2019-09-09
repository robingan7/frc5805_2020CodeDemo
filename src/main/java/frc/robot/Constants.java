package frc.robot;

public class Constants{

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    
    //--------cycle constant---------
    public static final double kLooperDt = 0.01;

     // Drive
     public static final int kLeftDriveMasterId = 12;
     public static final int kLeftDriveSlaveAId = 14;
     public static final int kLeftDriveSlaveBId = 16;
     public static final int kRightDriveMasterId = 11;
     public static final int kRightDriveSlaveAId = 13;
     public static final int kRightDriveSlaveBId = 15;
 
     // Followers
     public static final int kFollowerLeftAChannelId = 2;
     public static final int kFollowerLeftBChannelId = 3;
     public static final int kFollowerRightAChannelId = 0;
     public static final int kFollowerRightBChannelId = 1;
     public static final int kFollowerRearAChannelId = 4;
     public static final int kFollowerRearBChannelId = 5;

     // Solenoids
    public static final int kShifterSolenoidId = 4; // PCM 0, Solenoid 4
    public static final int kIntakeCloseSolenoid = 10;
    public static final int kIntakeClampSolenoid = 9;
    public static final int kForkliftDeploySolenoid = 7;  // CURRENTLY 6 ON PRACTICE!!!
    public static final int kFollowerWheelSolenoid = 11;
    public static final int kElevatorShifterSolenoidId = 8;
    public static final int kUnlockHookSolenoid = 4;
    public static final int kJazzHandsSolenoid = 5;
    public static final int kKickstandSolenoid = 3;

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    //Joystick value

    public static final int driveJoystickid=0;

    public static class TalonSRXConstants {
        public int id = -1;
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
    }

    public static class SuperStructureSubsystemConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public TalonSRXConstants kMasterConstants = new TalonSRXConstants();
        public TalonSRXConstants[] kSlaveConstants = new TalonSRXConstants[0];

        public double kHomePosition = 0.0; // Units
        public double kTicksPerUnitDistance = 1.0;
        public double kKp = 0;  // Raw output / raw error
        public double kKi = 0;  // Raw output / sum of raw error
        public double kKd = 0;  // Raw output / (err - prevErr)
        public double kKf = 0;  // Raw output / velocity in ticks/100ms
        public double kKa = 0;  // Raw output / accel in (ticks/100ms) / s
        public double kMaxIntegralAccumulator = 0;
        public int kIZone = 0; // Ticks
        public int kDeadband = 0; // Ticks

        public double kPositionKp = 0;
        public double kPositionKi = 0;
        public double kPositionKd = 0;
        public double kPositionKf = 0;
        public double kPositionMaxIntegralAccumulator = 0;
        public int kPositionIZone = 0; // Ticks
        public int kPositionDeadband = 0; // Ticks

        public int kCruiseVelocity = 0; // Ticks / 100ms
        public int kAcceleration = 0; // Ticks / 100ms / s
        public double kRampRate = 0.0; // s
        public int kContinuousCurrentLimit = 20; // amps
        public int kPeakCurrentLimit = 60; // amps
        public int kPeakCurrentDuration = 200; // milliseconds
        public double kMaxVoltage = 12.0;

        public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
        public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

        public int kStastusFrame8UpdateRate = 1000;
        public boolean kRecoverPositionOnReset = false;
    }

}