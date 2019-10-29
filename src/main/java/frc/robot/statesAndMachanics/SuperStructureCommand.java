package frc.robot.statesAndMachanics;

import frc.robot.subsystem.Wrist;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.SuperStructure;

import static frc.robot.Constants.SuperStructureConstants;

public class SuperStructureCommand{
    private static boolean mCargoShipPosition = false;
    private static boolean mMiddlePosition = false;
    private static boolean mHighPosition = false;
    private static boolean mLowPosition = false;

    //arm instance for updating value
    private static Arm arm_ = Arm.getInstance();
    private static Wrist wrist_ = Wrist.getInstance();

    //PID value
    private static SuperStructureState ScoreDiskLowFront;
    private static SuperStructureState ScoreDiskMiddleFront;
    private static SuperStructureState ScoreDiskHighFront;

    private static SuperStructureState ScoreDiskLowBack;
    private static SuperStructureState ScoreDiskMiddleBack;
    private static SuperStructureState ScoreDiskHighBack;

    private static SuperStructureState Defense;

    static{
        int level1 = arm_.getLevelOne();
        int facefront = wrist_.getFaceFront();
        int faceback = facefront + SuperStructureConstants.faceback_from_facefront;

        ScoreDiskLowFront = new SuperStructureState(level1, facefront);
        ScoreDiskMiddleFront = new SuperStructureState(level1 + SuperStructureConstants.frontlvl2_from_lvl1, facefront);
        ScoreDiskHighFront = new SuperStructureState(level1 + SuperStructureConstants.frontlvl3_from_lvl1, facefront);

        ScoreDiskLowBack = new SuperStructureState(level1 + SuperStructureConstants.backlvl1_from_lvl1, faceback);
        ScoreDiskMiddleBack = new SuperStructureState(level1 + SuperStructureConstants.backlvl2_from_lvl1, faceback);
        ScoreDiskHighBack = new SuperStructureState(level1 + SuperStructureConstants.backlvl3_from_lvl1, faceback);

        //defense_from_facefront: we don't know yet
        Defense = new SuperStructureState(level1 + SuperStructureConstants.defense_from_lvl1, facefront);
    }

    private SuperStructureCommand(){}

    public static void goToScoreDiskLow(boolean isBack) {
        if(isBack){
            selectPosition(ScoreDiskLowBack);
        } else{
            selectPosition(ScoreDiskLowFront);
        }
        
        mLowPosition = true;
    }

    public static void goToScoreDiskMiddle(boolean isBack) {
        if(isBack){
            selectPosition(ScoreDiskMiddleBack);
        } else{
            selectPosition(ScoreDiskMiddleFront);
        }
        
        mMiddlePosition = true;
    }

    public static void goToScoreDiskHigh(boolean isBack) {
        if(isBack){
            selectPosition(ScoreDiskHighBack);
        } else{
            selectPosition(ScoreDiskHighFront);
        }
        
        mHighPosition = true;
    }

    private static void selectPosition(SuperStructureState withDisk) {

        sendCommandToSuperstructure(withDisk);

        mCargoShipPosition = false;
        mMiddlePosition = false;
        mHighPosition = false;
        mLowPosition = false;
    }

    private static void sendCommandToSuperstructure(SuperStructureState position) {
        SuperStructure ss = SuperStructure.getInstance();
        ss.setGoal(new SuperStructureGoal(position));
    }

}