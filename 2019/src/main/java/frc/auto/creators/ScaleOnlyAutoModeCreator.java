package frc.auto.creators;

import frc.robot.AutoFieldState;
import frc.auto.AutoModeBase;
import frc.auto.modes.FarScaleOnlyMode;
import frc.auto.modes.NearScaleOnlyMode;

public class ScaleOnlyAutoModeCreator implements AutoModeCreator {

    private final AutoModeBase hardScaleMode;
    private final AutoModeBase easyScaleMode;
    private boolean mRobotStartedOnLeft;

    public ScaleOnlyAutoModeCreator(boolean robotStartedOnLeft) {
        mRobotStartedOnLeft = robotStartedOnLeft;
        hardScaleMode = new FarScaleOnlyMode(robotStartedOnLeft);
        easyScaleMode = new NearScaleOnlyMode(robotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        System.out.print("Getting ScaleOnlyAutoMode for " + mRobotStartedOnLeft + " / " + fieldState.toString());
        if ((mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.LEFT) ||
                (!mRobotStartedOnLeft && fieldState.getScaleSide() == AutoFieldState.Side.RIGHT)) {
            return easyScaleMode;
        } else {
            return hardScaleMode;
        }
    }
}
