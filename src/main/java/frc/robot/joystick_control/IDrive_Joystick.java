package frc.robot.joystick_control;

public interface IDrive_Joystick{
    double getSpeed();

    double getTurn();

    boolean getQuickTurn();

    boolean getShiftGear();

    boolean getClimbAuto();

    boolean getBackLeg();

    boolean getFrontLeg();
}