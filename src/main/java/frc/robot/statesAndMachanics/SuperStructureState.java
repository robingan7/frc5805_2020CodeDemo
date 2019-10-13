package frc.robot.statesAndMachanics;

public class SuperStructureState {
    public double arm_;
    public double wrist_;

    public SuperStructureState(double arm, double wrist){
        arm_ = arm;
        wrist_ = wrist;
    }

    public SuperStructureState(SuperStructureState other){
        arm_ = other.arm_;
        wrist_ = other.wrist_;
    }

    public SuperStructureState(){
        this(0,0);
    }

    public void setState(SuperStructureState other){
        arm_= other.arm_;
        wrist_ = other.wrist_;
    }
}