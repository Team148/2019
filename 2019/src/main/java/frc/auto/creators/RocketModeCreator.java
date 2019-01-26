package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.RocketMode;

public class RocketModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase RocketLeft;
    private AutoModeBase RocketRight;
    
    public RocketModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        RocketLeft = new RocketMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
        RocketRight = new RocketMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLeft) {
            return RocketLeft;
        }
        else {
            return RocketRight;
        }
    }
}