package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipFrontModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToCargoOneLineupForward;

    private DriveTrajectory mEndCargoOneToLoadingStation;

    private DriveTrajectory mLoadingStationToCargoTwo;

    public CargoShipFrontModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            mLevel1ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoOneLineupForwardLeft, true, false, TrajectoryGenerator.kCargoOneLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), true);
            mEndCargoOneToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonLeft, false, false, TrajectoryGenerator.kLoadingStationLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), true);
            mLoadingStationToCargoTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoTwoLineupLeft, true, false);
        
        }
        else {
            mLevel1ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoOneLineupForwardRight, true, false, TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
            mEndCargoOneToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoOneToLoadingStatonRight, false, false, TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
            mLoadingStationToCargoTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoTwoLineupRight, true, false, TrajectoryGenerator.kLoadingToCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -10.0)), TrajectoryGenerator.kLoadingToCargoTwoLineupPose.getTranslation().translateBy(new Translation2d(4.0, 10.0)), false);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo AND Rocket Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel1ToCargoOneLineupForward
                )),
                new DriveForwardAndTurnToTarget(20.0, 0.5),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(240.0), 0.75)
            )
        ));

        //Score Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoOneToLoadingStation,
                new DriveForwardAndTurnToTarget(20.0, 0.5),
                new OpenCloseBeak(false)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                mLoadingStationToCargoTwo,
                new TurnToHeading(Rotation2d.fromDegrees(90.0)),
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
                // new OpenLoopDrive(-0.5, -0.5, 0.25),
                // new TurnToHeading(Rotation2d.fromDegrees(0.0))
            )
        ));
    }
}