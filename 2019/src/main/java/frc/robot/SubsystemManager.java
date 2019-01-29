package frc.robot;

import frc.loops.ILooper;
import frc.loops.Loop;
import frc.loops.Looper;
import frc.robot.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

    private final List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> list) {
        mAllSubsystems = list;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void writeToLog() {
        mAllSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop() {
        mAllSubsystems.forEach((s) -> s.stop());
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for (Loop l : mLoops) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (Loop l : mLoops) {
                l.onLoop(timestamp);
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {
            for (Loop l : mLoops) {
                l.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}