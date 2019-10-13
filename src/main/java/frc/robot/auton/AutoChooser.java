package frc.robot.auton;

import frc.robot.auton.autoOptions.AutoOptionBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoChooser{
    enum StartingPosition {
        LEFT_HAB_2, RIGHT_HAB_2, CENTER_HAB_1, LEFT_HAB_1, RIGHT_HAB_1
    }

    enum DesiredMode {
        DRIVE_BY_CAMERA,
        LOW_ROCKET,
        SIDE_CARGO_SHIP_HATCH,
        FRONT_THEN_SIDE_CARGO_SHIP,
        DO_NOTHING
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> modeChooser_;
    private SendableChooser<StartingPosition> startPositionChooser_;

    private Optional<AutoOptionBase> autoOption_ = Optional.empty();

    public AutoChooser() {
        startPositionChooser_ = new SendableChooser<>();
        startPositionChooser_.setDefaultOption("Left HAB 2", StartingPosition.LEFT_HAB_2);
        startPositionChooser_.addOption("Right HAB 2", StartingPosition.RIGHT_HAB_2);
        startPositionChooser_.addOption("Right HAB 1", StartingPosition.RIGHT_HAB_1);
        startPositionChooser_.addOption("Left HAB 1", StartingPosition.LEFT_HAB_1);
        startPositionChooser_.addOption("Center HAB 1", StartingPosition.CENTER_HAB_1);

        SmartDashboard.putData("Starting Position", startPositionChooser_);

        modeChooser_ = new SendableChooser<>();
        modeChooser_.setDefaultOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        modeChooser_.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        modeChooser_.addOption("Rocket - LOW", DesiredMode.LOW_ROCKET);
        modeChooser_.addOption("Cargo Ship 2 Hatch", DesiredMode.SIDE_CARGO_SHIP_HATCH);
        modeChooser_.addOption("Front Then Side Cargo Ship",
                DesiredMode.FRONT_THEN_SIDE_CARGO_SHIP);
        SmartDashboard.putData("Auto mode", modeChooser_);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = modeChooser_.getSelected();
        StartingPosition staringPosition = startPositionChooser_.getSelected();
        if (mCachedDesiredMode != desiredMode || staringPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + staringPosition.name());
            autoOption_ = getAutoModeForParams(desiredMode, staringPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = staringPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.LEFT_HAB_2;
    }

    private boolean startingHab1(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.RIGHT_HAB_1;
    }

    private Optional<AutoOptionBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        /*
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case DRIVE_BY_CAMERA:
                return Optional.of(new DriveByCameraMode());
            case SIDE_CARGO_SHIP_HATCH:
                return Optional.of(new SideCargoShipHatchMode(startingLeft(position),
                        startingHab1(position)));
            case FRONT_THEN_SIDE_CARGO_SHIP:
                return Optional.of(new FrontThenSideCargoShipMode(
                        startingLeft(position), startingHab1(position)));
            case LOW_ROCKET:
                return Optional.of(new RocketHatchMode(startingLeft(position), startingHab1(position)));
            case TEST_CONTROL_FLOW:
                return Optional.of(new TestControlFlowMode());
            case DRIVE_CHARACTERIZATION_STRAIGHT:
                return Optional.of(new CharacterizeDrivebaseMode(true, false, false));
            case DRIVE_CHARACTERIZATION_TURN:
                return Optional.of(new CharacterizeDrivebaseMode(true, false, true));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);*/
        return Optional.empty();
    }

    public void reset() {
        autoOption_ = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoOptionBase> getAutoMode() {
        return autoOption_;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }
}