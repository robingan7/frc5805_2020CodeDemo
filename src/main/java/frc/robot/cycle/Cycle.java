package frc.robot.cycle;

public interface Cycle{
    void onStart(double time);
    void onLoop(double time);
    void onStop(double time);
}