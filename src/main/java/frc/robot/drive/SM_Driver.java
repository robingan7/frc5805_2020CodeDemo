package frc.robot.drive;

import frc.lib.utility.DriveSignal;

public class SM_Driver{
    public DriveSignal smDrive(double speed, double turn, boolean isQuickTurn){

        if(isQuickTurn){
            return new DriveSignal(speed, turn * 1.2);
        }
        return new DriveSignal(speed, turn);
    }
}