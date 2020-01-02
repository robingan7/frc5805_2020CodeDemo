package frc.robot.auton.autoOptions;

import frc.robot.statesAndMachanics.RobotState;
import frc.robot.auton.AutoEndEarlyException;
import frc.robot.action.*;
import frc.robot.paths.*;
import frc.robot.statesAndMachanics.SuperStructureCommand;
import frc.robot.subsystem.Drivebase;
import frc.robot.subsystem.SuperStructure;
import frc.lib.waypoint.Pose2D;
import frc.lib.waypoint.Rotation2D;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class SideCargoShipHatchMode extends AutoOptionBase {
    DrivePathAction first_path;
    DrivePathAction second_path;
    DrivePathAction third_path;
    DrivePathAction fourth_path;
    DrivePathAction fifth_path;
    boolean mLeft;
    boolean mStartHab1;

    public SideCargoShipHatchMode(boolean left, boolean startHab1) {
        mLeft = left;
        mStartHab1 = startHab1;
        if (mStartHab1) {
            first_path = new DrivePathAction(new Hab1ToCargoShip1Path(left));
        } else {
            first_path = new DrivePathAction(new GetOffHab2Path());
            second_path = new DrivePathAction(new Hab1ToCargoShip1Path(left));
        }
        third_path = new DrivePathAction(new CargoShip1ToFeederPath(left), false);
        fourth_path = new DrivePathAction(new FeederToCargoShip2Path(left));
    }

    @Override
    protected void routine() throws AutoEndEarlyException {
        if (mStartHab1) {
            runAction(new ParallelAction(Arrays.asList(
                    first_path,
                    new SeriesAction(Arrays.asList(
                            new LambdaAction(() -> SuperStructureCommand.goToScoreDiskLow(true))
                           )))));
        } else {
            runAction(
                    new ParallelAction(Arrays.asList(
                            first_path,
                            new LambdaAction(() -> SuperStructureCommand.goToScoreDiskLow(true)))));
            runAction(new DriveOpenLoopAction(-0.15, -0.15, 0.725));
            runAction(new LambdaAction(() ->
                    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2D.identity(), Rotation2D.identity())
            ));

            runAction(new LambdaAction(() -> Drivebase.getInstance().setHeading(Rotation2D.identity())));
            
            runAction(new ParallelAction(Arrays.asList(second_path,
                    new SeriesAction(Arrays.asList(
                            new WaitForPathMarkerAction(Hab1ToCargoShip1Path.kStartAutoAimingMarker))))));
        }
        RobotState.getInstance().resetVision();
        //SuperStructure.getInstance().resetAimingParameters();
        //runAction(new AutoAimAndScoreAction(Rotation2D.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0)));

        //SuperStructureCommand.goToPickupDiskFromWallFront();

        runAction(new ParallelAction(Arrays.asList(third_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(CargoShip1ToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()),
                //new LambdaAction(() -> SuperStructure.getInstance().resetAimingParameters()),
                new WaitUntilSeesTargetAction(), new ForceEndPathAction())))));

        //Action autosteerAction = new AutoSteerAndIntakeAction(/*reverse=*/true, false);
        //runAction(autosteerAction);

        //final Rotation2D rotation_hint = Rotation2D.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0);
        runAction(new ParallelAction(Arrays.asList(fourth_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(FeederToCargoShip3Path.kTurnTurretMarker),
                new LambdaAction(
                        () -> SuperStructureCommand.goToScoreDiskLow(false))
               )))));
        RobotState.getInstance().resetVision();
        //SuperStructure.getInstance().resetAimingParameters();
        //SuperStructure.getInstance().setWantAutoAim(
                //Rotation2D.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0));
    }
}