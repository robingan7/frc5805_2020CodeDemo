package frc.robot;

import frc.robot.cycle.*;
import frc.robot.Subsystem_Function;

import java.util.List;
import java.util.ArrayList;

public class Subsystem_Cycle_Manager implements ICycle_in{
   
    private final List<Subsystem_Function> mAllSubsystems;
    private List<Cycle> mCycle = new ArrayList<>();

    public Subsystem_Cycle_Manager(List<Subsystem_Function> allSubsystems) {
        mAllSubsystems = allSubsystems;
    }

    //---------EnabledLoop-----------
    private class EnabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {
            for (Cycle l : mCycle) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem_Function s : mAllSubsystems) {
                s.update_subsystem();
            }
            for (Cycle l : mCycle) {
                l.onLoop(timestamp);
            }
            for (Subsystem_Function s : mAllSubsystems) {
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
    private class DisabledLoop implements Cycle {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem_Function s : mAllSubsystems) {
                s.update_subsystem();
            }
            for (Subsystem_Function s : mAllSubsystems) {
                s.move_subsystem();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    }

    public void registerEnabledLoops(Cycle_in enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.enableSubsystem(new EnabledLoop());
    }

    public void registerDisabledLoops(Cycle_in disabledLooper) {
        disabledLooper.enableSubsystem(new DisabledLoop());
    }

    @Override
    public void enableSubsystem(Cycle loop) {
        mCycle.add(loop);
    }

}