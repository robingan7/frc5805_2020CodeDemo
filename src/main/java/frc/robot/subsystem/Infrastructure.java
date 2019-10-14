package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.cycle.*;

public class Infrastructure extends Subsystem_Function {
    private static Infrastructure instance_;

    private SuperStructureSubsystemContainer superStructure_ = SuperStructureSubsystemContainer.getInstance();
    private Compressor compressor_ = new Compressor();

    private boolean mIsManualControl = false;

    private Infrastructure() {}

    public static Infrastructure getInstance() {
        if (instance_ == null) {
            instance_ = new Infrastructure();
        }

        return instance_;
    }

    @Override
    public void registerEnabledLoops(ICycle_in mEnabledLooper) {
        mEnabledLooper.addSubsystem(new Cycle(){
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                synchronized (Infrastructure.this) {
                    boolean superstructureMoving = !superStructure_.isAtDesiredState();

                    if (superstructureMoving || !mIsManualControl) {
                        stopCompressor();
                    } else {
                        startCompressor();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    public synchronized void setIsManualControl(boolean isManualControl) {
        mIsManualControl = isManualControl;

        if (mIsManualControl) {
            startCompressor();
        }
    }

    public synchronized boolean isManualControl() {
        return mIsManualControl;
    }

    private void startCompressor() {
        compressor_.start();
    }

    private void stopCompressor() {
        compressor_.stop();
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSubsystem() {
        return false;
    }

}