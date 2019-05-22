package frc.lib.utility;

public abstract class CrashTrackerRunnable implements Runnable{
    @Override
    public final void run(){
        try{
            runCrashTracker();
        }catch(Throwable t){
            //CrashTracker.logT;
            throw t;
        }
    }

    public abstract void runCrashTracker();
}