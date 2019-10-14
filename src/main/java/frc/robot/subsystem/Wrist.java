package frc.robot.subsystem;


import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import frc.robot.Constants;
import frc.lib.motor.MotorUtil;

public class Wrist extends SuperStructureComponenet{
    private static Wrist instance_;
    private int facefront;

    public synchronized static Wrist getInstance(){
        if (instance_ == null) {
            instance_ = new Wrist(Constants.kWrist);
        }

        return instance_;
    }

    public Wrist(final Constants.SuperStructurComponentConstants constant){
        super(constant);   
        facefront = master_.getSelectedSensorPosition();
    }

    public int getFaceFront(){
        return facefront;
    }
    public double getAngle(){
        return getPosition();
    }

    @Override
    public boolean checkSubsystem(){
        return true;
    }

}