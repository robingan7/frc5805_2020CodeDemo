package frc.robot.subsystem;


import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import frc.robot.Constants;
import frc.lib.motor.MotorUtil;

public class Wrist extends SuperStructureComponenet{
    private static Wrist instance_ = new Wrist(Constants.kWrist);

    public synchronized static Wrist getInstance(){
        return instance_;
    }

    public Wrist(final Constants.SuperStructurComponentConstants constant ){
        super(constant);

        MotorUtil.checkError(master_.configRemoteFeedbackFilter(Constants.kCanifierWristId, RemoteSensorSource.CANifier_Quadrature,
                0, Constants.kLongCANTimeoutMs),
                "Could not set wrist encoder!!!: ");

        MotorUtil.checkError(master_.configSelectedFeedbackSensor(
                RemoteFeedbackDevice.RemoteSensor0, 0, Constants.kLongCANTimeoutMs),
                "Could not detect wrist encoder: ");
    }

    public double getAngle(){
        double result = 0;
        return result;
    }

    @Override
    public boolean checkSubsystem(){
        return true;
    }

}