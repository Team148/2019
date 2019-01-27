package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipMode;

public class CargoShipModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipLevel1;
    private AutoModeBase CargoShipLevel2;
    
    public CargoShipModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipLevel1 = new CargoShipMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
        CargoShipLevel2 = new CargoShipMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLevel1) {
            return CargoShipLevel1;
        }
        else {
            return CargoShipLevel2;
        }
    }
}
