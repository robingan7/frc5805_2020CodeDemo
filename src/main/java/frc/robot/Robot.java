package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.cycle.Cycle_in;
import frc.robot.drive.Drivebase;
import frc.robot.drive.SM_Driver;
import frc.robot.joystick_control.MainControlBoard;
import frc.lib.utility.DriveSignal;
import frc.robot.subsystem.*;
import frc.robot.auton.AutoActivator;
import frc.robot.auton.AutoChooser;
import frc.robot.auton.autoOptions.AutoOptionBase;
import frc.robot.statesAndMachanics.SuperStructureCommand;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
  private Cycle_in enabledLooper_ = new Cycle_in();
  private Cycle_in disabledLooper_ = new Cycle_in();

  private SM_Driver sm_driver_ = new SM_Driver();
  private MainControlBoard mControlBoard = MainControlBoard.getInstance();

  private final Drivebase drivebase_ = Drivebase.getInstance();
  private final Arm arm_ = Arm.getInstance();
  private final Wrist wrist_ = Wrist.getInstance();
  private final Infrastructure infrastructure_ = Infrastructure.getInstance();

  private final Subsystem_Cycle_Manager subsystem_Cycle_Manager_ = new Subsystem_Cycle_Manager(
    Arrays.asList(
      drivebase_,
      arm_,
      wrist_,
      infrastructure_
    )
  );

  private AutoChooser autoModeChooser_ = new AutoChooser();
  private AutoActivator autoModeActivator_;
  private boolean driveByCameraInAuto_ = false;
  
  @Override
  public void robotInit() {
    try{
      arm_.resetSensors();
      wrist_.resetSensors();
      drivebase_.resetSensors();

      subsystem_Cycle_Manager_.registerEnabledLoops(enabledLooper_);
      subsystem_Cycle_Manager_.registerDisabledLoops(disabledLooper_);

      autoModeChooser_.updateModeCreator();
    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    try{
      infrastructure_.setIsManualControl(true); 
    }catch(Throwable t){
      throw t;
    }
   
  }
  @Override
  public void autonomousPeriodic() {
    
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
    double speed = mControlBoard.getDriverJoystick().getSpeed();
    double turn = mControlBoard.getDriverJoystick().getTurn();
    boolean quickturn = mControlBoard.getDriverJoystick().getQuickTurn();
    boolean isArmBack = mControlBoard.getOperatorJoystick().isBack();
    boolean isOpenMani =  mControlBoard.getOperatorJoystick().isOpenManipulator();
    boolean isChangingGear = mControlBoard.getDriverJoystick().getShiftGear();
    boolean isLiftingFront = mControlBoard.getDriverJoystick().getFrontLeg();
    boolean isLiftingBack = mControlBoard.getDriverJoystick().getBackLeg();

    if(mControlBoard.getOperatorJoystick().isLvl1()){
      SuperStructureCommand.goToScoreDiskLow(isArmBack);
    } else if(mControlBoard.getOperatorJoystick().isLvl2()){
      SuperStructureCommand.goToScoreDiskMiddle(isArmBack);
    } else if(mControlBoard.getOperatorJoystick().isLvl3()){
      SuperStructureCommand.goToScoreDiskHigh(isArmBack);
    }

    if(isChangingGear){
      drivebase_.setHighGear();
    }

    if(isLiftingFront){
      drivebase_.setFrontLifter();
    }

    if(isLiftingBack){
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
  public void disabledPeriodic(){
    try{
      // Update auto modes
      autoModeChooser_.updateModeCreator();

      Optional<AutoOptionBase> autoMode = autoModeChooser_.getAutoMode();
      driveByCameraInAuto_ = autoModeChooser_.isDriveByCamera();
    }catch (Throwable t) {
        throw t;
    }
  }
}