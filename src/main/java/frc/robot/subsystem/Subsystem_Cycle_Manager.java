package frc.robot.subsystem;

import frc.robot.cycle.*;

import java.util.List;
import java.util.ArrayList;

/**
 * cycle_in class that contains multiple cycles
 * provides actual implementation for cycle
 */
public class Subsystem_Cycle_Manager implements ICycle_in{
   
    private final List<Subsystem_Function> allSubsystems_;//includes subsystems themselves
    private List<Cycle> cycles_ = new ArrayList<>();//includes cycles in subsystems

    public Subsystem_Cycle_Manager(List<Subsystem_Function> allSubsystems) {
        allSubsystems_ = allSubsystems;
    }

    //---------EnabledLoop-----------
    //provides actual implementation for cycle
    private class EnabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {
            cycles_.forEach(c -> c.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            allSubsystems_.forEach(Subsystem_Function::update_subsystem);
            cycles_.forEach(c -> c.onLoop(timestamp));
            allSubsystems_.forEach(Subsystem_Function::move_subsystem);
        }
        @Override
        public void onStop(double timestamp) {
            cycles_.forEach(c -> c.onStop(timestamp));
        }
    }

     //---------DisabledLoop-----------
     //provides actual implementation for cycle
    private class DisabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            allSubsystems_.forEach(Subsystem_Function::update_subsystem);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Cycle_in enabledLooper) {
        allSubsystems_.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.addSubsystem(new EnabledLoop());
    }

    public void registerDisabledLoops(Cycle_in disabledLooper) {
        disabledLooper.addSubsystem(new DisabledLoop());
    }

    @Override
    public void addSubsystem(Cycle loop) {
        cycles_.add(loop);
    }

    @Override
    public void sendAllDataToSmartDashboard(){
        allSubsystems_.forEach(Subsystem_Function::sendDataToSmartDashboard);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem_Function s : allSubsystems_) {
            ret_val &= s.checkSubsystem();
        }

        return ret_val;
    }


}