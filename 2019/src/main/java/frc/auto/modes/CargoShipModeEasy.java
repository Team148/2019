package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToCargoThree;
    private DriveTrajectory mCargoThreeTo90Turn;
    private DriveTrajectory mEndTurnToLoadingStation;
    private DriveTrajectory mLoadingStationToCargoTwo;

    public CargoShipModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        mLevel1ToCargoThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToCargoThree.get(mStartedLeft), true);
        mCargoThreeTo90Turn = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeTo90Turn.get(mStartedLeft));
        mEndTurnToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endTurnToLoadingStation.get(mStartedLeft));
        mLoadingStationToCargoTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoTwo.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                mLevel1ToCargoThree
            )
        ));

        //Turn 90 degrees
        runAction(new ParallelAction (
            Arrays.asList(
                mCargoThreeTo90Turn
            )
        ));

        //Get Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                mEndTurnToLoadingStation
            )
        ));

        //Score Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                mLoadingStationToCargoTwo
            )
        ));

        //Score Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                new OpenLoopDrive(0.5, 0.5, 0.2, false)
            )
        ));
    }
}