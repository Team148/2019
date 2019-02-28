package frc.auto.actions;

import frc.robot.subsystems.Beak;

public class OpenCloseBeak implements Action {
    private boolean mOpen;

    public OpenCloseBeak(boolean open) {
        mOpen = open;
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
        if (mOpen) {
            Beak.getInstance().setBeakGrab(true);
        } else {
            Beak.getInstance().setBeakGrab(false);
        }
    }
}
