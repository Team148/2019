package frc.auto.creators;

import frc.robot.AutoFieldState;
import frc.auto.AutoModeBase;
import frc.auto.modes.SwitchAndScaleMode;

public class SwitchAndScaleAutoModeCreator implements AutoModeCreator {

    // Pre-build trajectories to go left and right
    private SwitchAndScaleMode mLeftSwitchLeftScale = new SwitchAndScaleMode(true, true);
    private SwitchAndScaleMode mRightSwitchRightScale = new SwitchAndScaleMode(false, false);
    private SwitchAndScaleMode mLeftSwitchRightScale = new SwitchAndScaleMode(true, false);
    private SwitchAndScaleMode mRightSwitchLeftScale = new SwitchAndScaleMode(false, true);

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        if (fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT) {
                return mLeftSwitchLeftScale;
            } else {
                return mLeftSwitchRightScale;
            }
        } else {
            if (fieldState.getScaleSide() == AutoFieldState.Side.LEFT) {
                return mRightSwitchLeftScale;
            } else {
                return mRightSwitchRightScale;
            }
        }
    }
}
