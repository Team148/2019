package frc.auto.creators;

import frc.auto.AutoModeBase;
import frc.auto.modes.CargoShipANDRocketModeEasy;
import frc.auto.modes.CargoShipANDRocketModeHard;

public class CargoShipANDRocketModeCreator implements AutoModeCreator {

    //Pre build trajectory to go left or right, start from level 1 or 2
    private boolean mRobotStartedOnLevel1;
    private boolean mRobotStartedOnLeft;

    private AutoModeBase CargoShipANDRocketLevel1;
    private AutoModeBase CargoShipANDRocketLevel2;
    
    public CargoShipANDRocketModeCreator(boolean startOnLevelOne, boolean startOnLeft) {
        mRobotStartedOnLevel1 = startOnLevelOne;
        mRobotStartedOnLeft = startOnLeft;

        CargoShipANDRocketLevel1 = new CargoShipANDRocketModeEasy(mRobotStartedOnLeft);
        CargoShipANDRocketLevel2 = new CargoShipANDRocketModeHard(mRobotStartedOnLeft);
    }

    @Override
    public AutoModeBase getStateDependentAutoMode() {
        // System.out.println("Getting CargoShipMode for " + mRobotStartedOnLevel1 + " AND " + mRobotStartedOnLeft);
        if (mRobotStartedOnLevel1) {
            return CargoShipANDRocketLevel1;
        }
        else {
            return CargoShipANDRocketLevel2;
        }
    }
}