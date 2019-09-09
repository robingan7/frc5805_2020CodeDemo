package frc.lib.motor;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation;


public class MotorUtil{
    private final static int kTimeoutMs = 100;

    public static class Configuration {
    }

    public static WPI_TalonSRX createTalon(int id) {
        WPI_TalonSRX talon = new LazyTalonSRX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.setNeutralMode(NeutralMode.Brake);

        return talon;
    }

    public static WPI_VictorSPX createVictor(int id, int master_id){
        WPI_VictorSPX victor= new WPI_VictorSPX(id);
        victor.set(ControlMode.Follower, master_id);

        victor.setNeutralMode(NeutralMode.Brake);

        return victor;
    }

    private class IllegalArgumentException extends Exception{
        private IllegalArgumentException(String error){
            super(error);
        }
    }

    public static Solenoid makeSolenoidFromId(int solenoidId) {
       
        return new Solenoid(solenoidId);
        
        //throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + errorCode, false);
        }
    }
}