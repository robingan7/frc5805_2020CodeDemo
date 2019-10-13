package frc.robot.subsystem;

import frc.robot.cycle.*;

import java.util.List;
import java.util.ArrayList;

/**
 * cycle_in class that contains multiple cycles
 * provides actual implementation for cycle
 */
public class Subsystem_Cycle_Manager implements ICycle_in{
   
    private final List<Subsystem_Function> allSubsystems_;
    private List<Cycle> mCycle = new ArrayList<>();

    public Subsystem_Cycle_Manager(List<Subsystem_Function> allSubsystems) {
        allSubsystems_ = allSubsystems;
    }

    //---------EnabledLoop-----------
    //provides actual implementation for cycle
    private class EnabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {
            for (Cycle l : mCycle) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem_Function s : allSubsystems_) {
                s.update_subsystem();
            }
            for (Cycle l : mCycle) {
                l.onLoop(timestamp);
            }
            for (Subsystem_Function s : allSubsystems_) {
                s.move_subsystem();
            }
        }

        @Override
        public void onStop(double timestamp) {
            for (Cycle l : mCycle) {
                l.onStop(timestamp);
            }
        }
    }

     //---------DisabledLoop-----------
     //provides actual implementation for cycle
    private class DisabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {
            
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem_Function s : allSubsystems_) {
                s.update_subsystem();
            }
            for (Subsystem_Function s : allSubsystems_) {
                s.move_subsystem();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
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
        mCycle.add(loop);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem_Function s : allSubsystems_) {
            ret_val &= s.checkSubsystem();
        }

        return ret_val;
    }


}