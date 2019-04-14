package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;

    private DriveTrajectory mLevel1ToCargoTwoLineupForward;

    private DriveTrajectory mCargoTwoAway;
    private DriveTrajectory mEndCargoTwoToLoadingStation;

    private DriveTrajectory mLoadingStationToCargoThreeLineup;

    public CargoShipModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForwardLeft, true);
        
            mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoLeft, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonLeft);

            mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupLeft, true);
        }
        else {
            mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForwardRight, true, false, TrajectoryGenerator.kCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
        
            mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoRight, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonRight);

            mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupRight, true);
        }

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel1ToCargoTwoLineupForward
                )),
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(210.0))
            )
        ));

        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mCargoTwoAway
        //     )
        // ));

        //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoTwoToLoadingStation
                // new DriveForwardAndTurnToTarget(20.0, 1.0),
                // new OpenCloseBeak(false),
                // new OpenLoopDrive(-0.15, -0.15, 0.5)
            )
        ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         mLoadingStationToCargoThreeLineup,
        //         new DriveForwardAndTurnToTarget(40.0, 1.0)
        //     )
        // ));
    }
}