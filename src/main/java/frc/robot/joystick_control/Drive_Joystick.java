package frc.robot.joystick_control;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;//WPILIB_USED_HERE

public class Drive_Joystick implements IDrive_Joystick{
    private static Drive_Joystick mInstance = new Drive_Joystick();

    public static Drive_Joystick getInstance(){
        return mInstance;
    }

    private Joystick mDriverJoystick;

    private Drive_Joystick(){
        mDriverJoystick = new Joystick(Constants.driveJoystickid);
    }

    @Override
    public double getSpeed(){
        return mDriverJoystick.getRawAxis(1);
    }

    @Override
    public double getTurn(){
        return mDriverJoystick.getRawAxis(4);
    }

    @Override
    public boolean getQuickTurn(){
        return mDriverJoystick.getRawButton(8);
    }

    @Override
    public boolean getShiftGear(){
        return mDriverJoystick.getRawButton(3);
    }

    @Override
    public boolean getClimbAuto(){
        return mDriverJoystick.getRawButton(5);
    }

    @Override
    public boolean getBackLeg(){
        return mDriverJoystick.getRawButton(6);
    }

    @Override
    public boolean getFrontLeg(){
        return mDriverJoystick.getRawButton(7);
    }
}