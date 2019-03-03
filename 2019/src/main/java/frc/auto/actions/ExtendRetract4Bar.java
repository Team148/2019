package frc.auto.actions;

import frc.robot.subsystems.Beak;

public class ExtendRetract4Bar implements Action {
    private boolean mOut;

    public ExtendRetract4Bar(boolean out) {
        mOut = out;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        if (mOut) {
            Beak.getInstance().setBeakBar(true);
        } else {
            Beak.getInstance().setBeakBar(false);
        }
    }
}
