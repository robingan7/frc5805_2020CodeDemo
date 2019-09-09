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
    private Cycle_in mEnabledLooper = new Cycle_in();
    private Cycle_in mDisabledLooper = new Cycle_in();
    private SM_Driver mSM_Driver = new SM_Driver();
    private MainControlBoard mControlBoard = MainControlBoard.getInstance();

    private final Subsystem_Cycle_Manager mSubsystem_Cycle_Manager = new Subsystem_Cycle_Manager(
      Arrays.asList(
        Drivebase.getInstance()
      )
    );

private Drivebase mDrive = Drivebase.getInstance();
  
  @Override
  public void robotInit() {
    try{
      mSubsystem_Cycle_Manager.registerEnabledLoops(mEnabledLooper);
      mSubsystem_Cycle_Manager.registerDisabledLoops(mDisabledLooper);
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
      mDisabledLooper.stop_all();
      mEnabledLooper.start_all();

      mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
      mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));//set a number that is less than the deadband
    }catch(Throwable t){
        throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
    double timestamp = Timer.getFPGATimestamp();
    double speed = mControlBoard.getDriverJoystick().getSpeed();
    double turn = mControlBoard.getDriverJoystick().getTurn();
    boolean quickturn=mControlBoard.getDriverJoystick().getQuickTurn();

    ///System.out.println(speed+" "+turn);
    try{
      mDrive.setOpenLoop(mSM_Driver.smDrive(speed, turn, quickturn));//return a drive signal class to set open loop

    }catch(Throwable t){
      throw t;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
