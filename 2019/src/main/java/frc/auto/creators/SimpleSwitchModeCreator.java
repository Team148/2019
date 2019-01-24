package frc.auto.creators;

import frc.robot.AutoFieldState;
import frc.auto.AutoModeBase;
import frc.auto.modes.SimpleSwitchMode;

public class SimpleSwitchModeCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private SimpleSwitchMode mGoLeftMode = new SimpleSwitchMode(true);
    private SimpleSwitchMode mGoRightMode = new SimpleSwitchMode(false);

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        if (fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
            return mGoLeftMode;
        } else {
            return mGoRightMode;
        }
    }
}
