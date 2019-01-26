package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipANDRocketMode;

public class CargoShipANDRocketModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipANDRocketLeft;
    private AutoModeBase CargoShipANDRocketRight;
    
    public CargoShipANDRocketModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipANDRocketLeft = new CargoShipANDRocketMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
        CargoShipANDRocketRight = new CargoShipANDRocketMode(mRobotStartedOnLevel1, mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLeft) {
            return CargoShipANDRocketLeft;
        }
        else {
            return CargoShipANDRocketRight;
        }
    }
}