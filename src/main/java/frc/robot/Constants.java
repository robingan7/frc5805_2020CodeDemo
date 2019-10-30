package frc.robot;

public class Constants{

    //motion profile solot
    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
    public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

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

    public static final int kGearShifter = 4;
    public static final int kFrontLifter = 3;
    public static final int kBackLifter = 5;

    //Wrist
    public static final int kCanifierWristId = 1;
    public static final int kGrabberF = 0;
    public static final int kGrabberR = 7; 

    //compressor
    
    // Followers
    public static final int kFollowerLeftAChannelId = 2;
    public static final int kFollowerLeftBChannelId = 3;
    public static final int kFollowerRightAChannelId = 0;
    public static final int kFollowerRightBChannelId = 1;
    public static final int kFollowerRearAChannelId = 4;
    public static final int kFollowerRearBChannelId = 5;   

    
    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 10; //use for constructors

    //Joystick value

    public static final int driveJoystickid = 0;
    public static final int operatorJoystickid = 1;

    public static class VictorSRXConstants {
        public int id = -1;
        public boolean invert_motor = false;
        public boolean invert_sensor_phase = false;
    }

    public static class SuperStructurComponentConstants {
        public String kName = "ERROR_ASSIGN_A_NAME";

        public VictorSRXConstants kMasterConstants = new VictorSRXConstants();
        public VictorSRXConstants[] kSlaveConstants = new VictorSRXConstants[0];

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

        public double kNominalOutputForward = 0;
        public double kNominalOutputReverse = 0;
    }

    // Superstructure Constants
    // arm
    public static final SuperStructurComponentConstants kArm = new SuperStructurComponentConstants();
    static {
        kArm.kName = "Arm";

        kArm.kMasterConstants.id = 18;
        kArm.kMasterConstants.invert_motor = false;
        kArm.kMasterConstants.invert_sensor_phase = false;
        kArm.kSlaveConstants = new VictorSRXConstants[1];

        kArm.kSlaveConstants[0] = new VictorSRXConstants();

        kArm.kSlaveConstants[0].id = 19;
        kArm.kSlaveConstants[0].invert_motor = false;

        kArm.kKp = 0.4;
        kArm.kKi = 0.0001;
        kArm.kKd = 0;
        kArm.kKf = 0;
        kArm.kKa = 0.0;
        kArm.kMaxIntegralAccumulator = 0;
        kArm.kIZone = 0; // Ticks
        kArm.kDeadband = 0; // Ticks

        kArm.kPositionMaxIntegralAccumulator = 0;
        kArm.kPositionIZone = 400; // Ticks
        kArm.kPositionDeadband = 0; // Ticks

        kArm.kMaxUnitsLimit = 31.1; // inches
        kArm.kMinUnitsLimit = 0.0; // inches

        kArm.kCruiseVelocity = 300; //should be 3000
        kArm.kAcceleration = 300; // should be 3000
        kArm.kRampRate = 0.005; // s
        kArm.kContinuousCurrentLimit = 14; // amps
        kArm.kPeakCurrentLimit = 40; // amps
        kArm.kPeakCurrentDuration = 14; // milliseconds

        kArm.kNominalOutputForward = 0;
        kArm.kNominalOutputReverse = 0;
    }

    //wrist constant
    public static final SuperStructurComponentConstants kWrist = new SuperStructurComponentConstants();
    static {
        kWrist.kName = "Wrist";

        kWrist.kMasterConstants.id = 17;
        kWrist.kMasterConstants.invert_motor = false;
        kWrist.kMasterConstants.invert_sensor_phase = false;
        kWrist.kSlaveConstants = new VictorSRXConstants[0];

        kWrist.kKp = 0.6;
        kWrist.kKi = 0.0001;
        kWrist.kKd = 0;
        kWrist.kKf = 0;
        kWrist.kKa = 0.0;
        kWrist.kMaxIntegralAccumulator = 0;
        kWrist.kIZone = 0; // Ticks
        kWrist.kDeadband = 0; // Ticks

        kWrist.kPositionMaxIntegralAccumulator = 0;
        kWrist.kPositionIZone = 400; // Ticks
        kWrist.kPositionDeadband = 0; // Ticks

        kWrist.kMaxUnitsLimit = 31.1; // inches
        kWrist.kMinUnitsLimit = 0.0; // inches

        kWrist.kCruiseVelocity = 200; // should be 2000
        kWrist.kAcceleration = 300; // should be 3000
        kWrist.kRampRate = 0.005; // s
        kWrist.kContinuousCurrentLimit = 20; // amps
        kWrist.kPeakCurrentLimit = 40; // amps
        kWrist.kPeakCurrentDuration = 20; // milliseconds

        kWrist.kNominalOutputForward = 0.1;
        kWrist.kNominalOutputReverse = 0.1;
    }

    //superstrcuture
    public static class SuperStructureConstants {
        public static final double kArmPaddingAbsolute = 200;
        public static final double kWristPaddingAbsolute = 100;
    
        public static final double[] kPadding = {kArmPaddingAbsolute, kWristPaddingAbsolute};

        //Arm position
        public static final int startmatch_from_level1 = 1407;
        public static final int vertex_from_lvl1 = 6235;
        public static final int reverselimit_from_lvl1 = -1205;
        public static final int forwardlimit_from_lvl1 = 13174;

        public static final int backlvl1_from_lvl1 = 12550;
        public static final int backlvl2_from_lvl1 = 10146;
        public static final int backlvl3_from_lvl1 = 7250;
        public static final int frontlvl3_from_lvl1 = 5518;
        public static final int frontlvl2_from_lvl1 = 2468;
        public static final int defense_from_lvl1 = -1700;

        //Wrist position
        public static final int faceback_from_facefront = 8376;
    }
}