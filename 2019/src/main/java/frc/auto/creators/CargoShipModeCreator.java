package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipMode;

public class CargoShipModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipLeft;
    private AutoModeBase CargoShipRight;
    
    public CargoShipModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipLeft = new CargoShipMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
        CargoShipRight = new CargoShipMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLeft) {
            return CargoShipLeft;
        }
        else {
            return CargoShipRight;
        }
    }
}
