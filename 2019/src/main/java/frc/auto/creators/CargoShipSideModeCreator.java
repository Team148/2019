package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipSideModeEasy;
import frc.auto.modes.CargoShipSideModeHard;

public class CargoShipSideModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipSideLevel1;
    private AutoModeBase CargoShipSideLevel2;
    
    public CargoShipSideModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipSideLevel1 = new CargoShipSideModeEasy(mRobotStartedOnLeft);
        CargoShipSideLevel2 = new CargoShipSideModeHard(mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        // System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLevel1) {
            return CargoShipSideLevel1;
        }
        else {
            return CargoShipSideLevel2;
        }
    }
}
