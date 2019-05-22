package frc.robot.cycle;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.lib.utility.CrashTrackerRunnable;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class Cycle_in implements ICycle_in{

    private boolean isRunning;

    public final double kPeriod = Constants.kLooperDt;
    private final Notifier notifier_;
    private final List<Cycle> cycles;
    private final Object taskCurrentRun_=new Object();
    private double timestamp_=0;
    private double dt_=0;

    private final CrashTrackerRunnable runnable_= new CrashTrackerRunnable(){
    
        @Override
        public void runCrashTracker() {
            synchronized(runnable_){
                if(isRunning){
                    double current_time=Timer.getFPGATimestamp();
                    for(Cycle cycle:cycles){
                        cycle.onLoop(current_time);
                    }
                    dt_=current_time-timestamp_;
                    timestamp_=current_time;
                }
            }
        }
    };

    public Cycle_in(){
        notifier_=new Notifier(runnable_);
        isRunning=false;
        cycles=new ArrayList<>();
    }

    @Override
    public synchronized void enableSubsystem(Cycle cycle){
        synchronized(cycle){
            cycles.add(cycle);
        }
    }

    public synchronized void start_all() {
        if (!isRunning) {
            System.out.println("Starting cycles");
            synchronized (taskCurrentRun_) {
                timestamp_ = Timer.getFPGATimestamp();
                for (Cycle cycle : cycles) {
                    cycle.onStart(timestamp_);
                }
                isRunning = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop_all() {
        if (isRunning) {
            System.out.println("Stopping cycles");
            notifier_.stop();
            synchronized (taskCurrentRun_) {
                isRunning = false;
                timestamp_ = Timer.getFPGATimestamp();
                for (Cycle cycle : cycles) {
                    System.out.println("Stopping " + cycle);
                    cycle.onStop(timestamp_);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("cycleer_dt", dt_);
    }
}