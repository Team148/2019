package frc.auto.actions;

import frc.robot.subsystems.FloorBallIntake;

public class ExtendRetractBallIntake implements Action {
    private boolean mOut;

    public ExtendRetractBallIntake(boolean out) {
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
            FloorBallIntake.getInstance().setBallIntakeCylinder(true);
        } else {
            FloorBallIntake.getInstance().setBallIntakeCylinder(false);
        }
    }
}