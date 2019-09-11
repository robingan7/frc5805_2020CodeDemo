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

import java.util.Arrays;

public class Robot extends TimedRobot {
  private Cycle_in enabledLooper_ = new Cycle_in();
  private Cycle_in disabledLooper_ = new Cycle_in();

  private SM_Driver sm_driver_ = new SM_Driver();
  private MainControlBoard mControlBoard = MainControlBoard.getInstance();

  private final Subsystem_Cycle_Manager subsystem_Cycle_Manager_ = new Subsystem_Cycle_Manager(
    Arrays.asList(
      Drivebase.getInstance(),
      Arm.getInstance()
    )
  );

  private Drivebase drivebase_ = Drivebase.getInstance();
  private final Arm arm_ = Arm.getInstance();
  
  @Override
  public void robotInit() {
    try{
     
      subsystem_Cycle_Manager_.registerEnabledLoops(enabledLooper_);
      subsystem_Cycle_Manager_.registerDisabledLoops(disabledLooper_);



    }catch(Throwable t){
      throw t;
    }
    
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
   
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
    boolean quickturn=mControlBoard.getDriverJoystick().getQuickTurn();

    drivebase_.setOpenLoop(sm_driver_.smDrive(speed, turn, quickturn));//return a drive signal class to set open loop
  }

  @Override
  public void testPeriodic() {
  }
}
