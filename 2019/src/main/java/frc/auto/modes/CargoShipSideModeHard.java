package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipSideModeHard extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private double angleToLoadingStation;

    private DriveTrajectory mLevel2ToCargoTwoLineupForward;

    // private DriveTrajectory mCargoTwoAway;

    private DriveTrajectory mEndCargoTwoToLoadingStation;

    // private DriveTrajectory mLoadingStationToCargoThreeLineup;

    public CargoShipSideModeHard(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            angleToLoadingStation = 150.0;

            mLevel2ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelTwoToCargoTwoLineupForwardLeft, true, false, TrajectoryGenerator.kCargoTwoLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoTwoLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
        
            // mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoLeft, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonLeft, false, false, TrajectoryGenerator.kLoadingStationLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kLoadingStationLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);

            // mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupLeft, true, false);
        }
        else {
            angleToLoadingStation = 210.0;

            mLevel2ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelTwoToCargoTwoLineupForwardRight, true, false, TrajectoryGenerator.kCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
        
            // mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoRight, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonRight, false, false, TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);

            // mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupRight, true, false);
        }

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Side Mode Hard");

        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel2ToCargoTwoLineupForward
                )),
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(angleToLoadingStation))
            )
        ));

        //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoTwoToLoadingStation,
                new DriveForwardAndTurnToTarget(20.0, 1.0)
            )
        ));
    }
}