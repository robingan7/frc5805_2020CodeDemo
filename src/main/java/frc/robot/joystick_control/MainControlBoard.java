package frc.robot.joystick_control;

public class MainControlBoard{// implements IMainControlBoard
    private static MainControlBoard mInstance = null;
    private IDrive_Joystick mDrive_Joystick;
    private IOperator_Joystick mIOperator_Joystick;

    public static MainControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainControlBoard();
        }
        return mInstance;
    }

    public MainControlBoard(){
        mDrive_Joystick=Drive_Joystick.getInstance();
        mIOperator_Joystick=Operator_Joystick.getInstance();
    }

    //methods below should override, I'm too lazy to override all methods so I didn't implement the interface
    public double getSpeed(){
        return mDrive_Joystick.getSpeed();
    }

    public double getTurn(){
        return mDrive_Joystick.getTurn();
    }

    public boolean getQuickTurn(){
        return mDrive_Joystick.getQuickTurn();
    }
}