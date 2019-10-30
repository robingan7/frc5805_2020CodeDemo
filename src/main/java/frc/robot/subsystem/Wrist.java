package frc.robot.subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import frc.robot.Constants;

public class Wrist extends SingleMasterMotorSystem{
    private static Wrist instance_;
    private int facefront = 0;
    private DoubleSolenoid grabber_;

    public synchronized static Wrist getInstance(){
        if (instance_ == null) {
            instance_ = new Wrist(Constants.kWrist);
        }

        return instance_;
    }

    public Wrist(final Constants.SuperStructurComponentConstants constant){
        super(constant);   
        facefront = master_.getSelectedSensorPosition();
        System.out.println("Face Front: " + facefront);
        System.out.println("Face Front2: " + master_.getSelectedSensorPosition());

        grabber_ = new DoubleSolenoid(Constants.kGrabberF, Constants.kGrabberR);
    }

    public int getFaceFront(){
        return facefront;
    }

    public void setGrabber(boolean state){
        if(state){
            grabber_.set(Value.kForward);
        }else{
            grabber_.set(Value.kReverse);
        }
    }
    
    @Override 
    public void sendDataToSmartDashboard(){
        SmartDashboard.putNumber("Wrist Value", master_.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Current:", master_.getOutputCurrent());
        SmartDashboard.putNumber("Wrist Error: ", master_.getClosedLoopError());
        SmartDashboard.putNumber("Wrist Demand: ",  feedData_.demand);
        SmartDashboard.putNumber("Wrist Percentage: ", master_.getMotorOutputPercent());
        SmartDashboard.putString("Wrist ControlMode: ", master_.getControlMode().toString());
        SmartDashboard.putNumber("Wrist Error: ", master_.getClosedLoopError());
        SmartDashboard.putNumber("Wrist start Value", facefront);
    }
    
    @Override
    public boolean checkSubsystem(){
        return true;
    }

}