package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipFrontModeEasy;
import frc.auto.modes.CargoShipFrontModeHard;

public class CargoShipFrontModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipFrontLevel1;
    private AutoModeBase CargoShipFrontLevel2;
    
    public CargoShipFrontModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipFrontLevel1 = new CargoShipFrontModeEasy(mRobotStartedOnLeft);
        CargoShipFrontLevel2 = new CargoShipFrontModeHard(mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        // System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLevel1) {
            return CargoShipFrontLevel1;
        }
        else {
            return CargoShipFrontLevel2;
        }
    }
}