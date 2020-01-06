package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.cycle.Cycle_in;
import frc.robot.cycle.Subsystem_Cycle_Manager;
import frc.robot.subsystem.Drivebase;
import frc.lib.utility.SM_Driver;
import frc.robot.joystick_control.MainControlBoard;
import frc.lib.utility.DriveSignal;
import frc.robot.subsystem.*;
import frc.robot.auton.AutoActivator;
import frc.robot.auton.AutoChooser;
import frc.robot.auton.autoOptions.AutoOptionBase;
import frc.robot.statesAndMachanics.SuperStructureCommand;
import frc.robot.statesAndMachanics.RobotState;
import frc.lib.waypoint.Rotation2D;
import frc.lib.waypoint.Pose2D;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Cycle_in enabledLooper_ = new Cycle_in();
  private Cycle_in disabledLooper_ = new Cycle_in();

  private SM_Driver sm_driver_ = new SM_Driver();
  private MainControlBoard controlBoard_ = MainControlBoard.getInstance();

  private final RobotState robotState_ = RobotState.getInstance();
  private final Arm arm_ = Arm.getInstance();
  private final Wrist wrist_ = Wrist.getInstance();
  private final Infrastructure infrastructure_ = Infrastructure.getInstance();
  private final SuperStructure superstructure_ = SuperStructure.getInstance();
  private final Drivebase drivebase_ = Drivebase.getInstance();
  private final Subsystem_Cycle_Manager subsystem_Cycle_Manager_ = new Subsystem_Cycle_Manager(
    Arrays.asList(
      arm_,
      wrist_,
      superstructure_,
      infrastructure_,
      drivebase_
    )
  );
  private AutoChooser autoModeChooser_ = new AutoChooser();
  private AutoActivator autoModeActivator_;
  
  @Override
  public void robotInit() {
    try{
      drivebase_.resetSensors();
      
      // Robot starts forwards.
      robotState_.reset(Timer.getFPGATimestamp(), Pose2D.getDefault(), Rotation2D.getDefault());
      drivebase_.setHeading(Rotation2D.getDefault());
 
      subsystem_Cycle_Manager_.registerEnabledLoops(enabledLooper_);
      subsystem_Cycle_Manager_.registerDisabledLoops(disabledLooper_);
      SuperStructureCommand.goToScoreDiskLow(false);

      autoModeChooser_.updateModeCreator();
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    try {
      subsystem_Cycle_Manager_.sendDataToSmartDashboard();
      robotState_.outputToSmartDashboard();
      autoModeChooser_.outputToSmartDashboard();
    } catch (Throwable t) {
        throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try{
      infrastructure_.setIsManualControl(true);

      // Robot starts forwards.
      robotState_.reset(Timer.getFPGATimestamp(), Pose2D.getDefault(), Rotation2D.getDefault());
      drivebase_.setHeading(Rotation2D.getDefault());

      Optional<AutoOptionBase> autoMode = autoModeChooser_.getAutoOption();
      if (autoMode.isPresent() && autoMode.get() != autoModeActivator_.getAutoOption()) {
          System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
          autoModeActivator_.setAutoOption(autoMode.get());
      }

      autoModeActivator_.start();
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
    try{
      teleopControl(true);
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Robot State", "Start driving");
    try{
      disabledLooper_.stop_all();
      enabledLooper_.start_all();
      infrastructure_.setIsManualControl(true); 

      drivebase_.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
      drivebase_.setOpenLoop(new DriveSignal(0.05, 0.05));//set a number that is less than the deadband
    }catch(Throwable t){
        throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
    try{
      teleopControl(false);
    }catch(Throwable t){
      throw t;
    }
  }

  public void teleopControl(boolean sandStorm){
    double timestamp = Timer.getFPGATimestamp();
    double speed = controlBoard_.getDriverJoystick().getSpeed();
    double turn = controlBoard_.getDriverJoystick().getTurn();
    boolean quickturn = controlBoard_.getDriverJoystick().getQuickTurn();
    boolean isChangingGear = controlBoard_.getDriverJoystick().getShiftGear();
    boolean isLiftingFront = controlBoard_.getDriverJoystick().getFrontLeg();
    boolean isLiftingBack = controlBoard_.getDriverJoystick().getBackLeg();

    boolean isArmBack = controlBoard_.getOperatorJoystick().isBack();
    boolean isOpenMani =  controlBoard_.getOperatorJoystick().isOpenManipulator();

    if(controlBoard_.getOperatorJoystick().isLvl1()){
      SuperStructureCommand.goToScoreDiskLow(isArmBack);

    } else if(controlBoard_.getOperatorJoystick().isLvl2()){
      SuperStructureCommand.goToScoreDiskMiddle(isArmBack);

    } else if(controlBoard_.getOperatorJoystick().isLvl3()){
      SuperStructureCommand.goToScoreDiskHigh(isArmBack);

    } else if(controlBoard_.getOperatorJoystick().isDefense()){
      SuperStructureCommand.goToDefense();
    }

    if(isChangingGear) {
      drivebase_.setHighGear();
    }

    if(isLiftingFront) {
      drivebase_.setFrontLifter();
    }

    if(isLiftingBack) {
      drivebase_.setBackLifter();
    }

    wrist_.setGrabber(isOpenMani);

    drivebase_.setOpenLoop(sm_driver_.smDrive(speed, turn, false));//return a drive signal class to set open loop
  }

  @Override
  public void testInit() {
    try {
      System.out.println("Starting check systems.");

      disabledLooper_.stop_all();
      enabledLooper_.stop_all();

      if (subsystem_Cycle_Manager_.checkSubsystems()) {
          System.out.println("ALL SYSTEMS PASSED");
      } else {
          System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
      }
    } catch (Throwable t) {
        throw t;
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() { 
    try{
      disabledLooper_.stop_all();

      if (autoModeActivator_ != null) {
          autoModeActivator_.stop();
      }

      autoModeChooser_.reset();
      autoModeChooser_.updateModeCreator();
      autoModeActivator_ = new AutoActivator();

      infrastructure_.setIsManualControl(false);
    }catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try{
      // Update auto modes
      autoModeChooser_.updateModeCreator();

      //Optional<AutoOptionBase> autoMode = autoModeChooser_.getAutoOption();
    }catch (Throwable t) {
        throw t;
    }
  }
}