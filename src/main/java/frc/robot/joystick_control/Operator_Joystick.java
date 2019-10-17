package frc.robot.joystick_control;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;
public class Operator_Joystick implements IOperator_Joystick{
    private static final Operator_Joystick mInstance = new Operator_Joystick();

    public static Operator_Joystick getInstance(){
        return mInstance;
    }

    private Joystick mOperatorJoystick;

    private Operator_Joystick(){
        mOperatorJoystick = new Joystick(Constants.operatorJoystickid);
    }

    @Override
    public boolean isOpenManipulator(){
        return mOperatorJoystick.getRawButton(7);
    }

    @Override
    public boolean isBack(){
        return mOperatorJoystick.getRawButton(5);
    }

    @Override
    public boolean isLvl3(){
        return mOperatorJoystick.getRawButton(1);
    }

    @Override
    public boolean isLvl2(){
        return mOperatorJoystick.getRawButton(2);
    }

    @Override
    public boolean isLvl1(){
        return mOperatorJoystick.getRawButton(3);
    }
}