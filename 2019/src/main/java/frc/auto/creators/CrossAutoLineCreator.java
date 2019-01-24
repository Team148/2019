package frc.auto.creators;

import frc.robot.AutoFieldState;
import frc.auto.AutoModeBase;
import frc.auto.modes.CrossAutoLineMode;

public class CrossAutoLineCreator implements AutoModeCreator {

    // Pre-build trajectories to go left and right
    private CrossAutoLineMode mCrossAutoLineMode = new CrossAutoLineMode();

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        return mCrossAutoLineMode;
    }
}
