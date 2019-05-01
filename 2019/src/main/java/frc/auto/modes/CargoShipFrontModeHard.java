package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipFrontModeHard extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    
    final boolean mStartedLeft;
    private double angleToLoadingStation;
    private double angleToCargoTwo;
    private double angleToCargoThree;

    private DriveTrajectory mLevel2ToCargoOneLineupForward;

    private DriveTrajectory mEndCargoOneToLoadingStation;

    private DriveTrajectory mLoadingStationToCargoTwo;

    public CargoShipFrontModeHard(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            angleToLoadingStation = 120.0;
            angleToCargoTwo = 270.0;
            angleToCargoThree = 315.0;

            mLevel2ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelTwoToCargoOneLineupForwardLeft, true, false, TrajectoryGenerator.kCargoOneLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -5.0)), TrajectoryGenerator.kCargoOneLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 5.0)), false);
            mEndCargoOneToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoOneToLoadingStatonLeft, true, false, TrajectoryGenerator.kLoadingStationLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -10.0)), TrajectoryGenerator.kLoadingStationLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 10.0)), false);
            mLoadingStationToCargoTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoTwoLineupLeft, true, false, TrajectoryGenerator.kLoadingToCargoTwoLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -10.0)), TrajectoryGenerator.kLoadingToCargoTwoLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 10.0)), false);
        
        }
        else {
            angleToLoadingStation = 240.0;
            angleToCargoTwo = 90.0;
            angleToCargoThree = 45.0;

            mLevel2ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelTwoToCargoOneLineupForwardRight, true, false, TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -10.0)), TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 10.0)), false);
            mEndCargoOneToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoOneToLoadingStatonRight, true, false, TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -10.0)), TrajectoryGenerator.kLoadingStationLineupPose.getTranslation().translateBy(new Translation2d(4.0, 10.0)), false);
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
                    mLevel2ToCargoOneLineupForward
                )),
                new DriveForwardAndTurnToTarget(20.0, 1.0),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(-20.0, 0.25),
                // new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(angleToLoadingStation), 0.75)
            )
        ));

        
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoOneToLoadingStation,
                new DriveForwardAndTurnToTarget(20.0, 1.0),
                new OpenCloseBeak(false)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                mLoadingStationToCargoTwo,
                new TurnToHeading(Rotation2d.fromDegrees(angleToCargoTwo), 1.0),
                new DriveForwardAndTurnToTarget(20.0, 1.75),
                new OpenCloseBeak(true)
            )
        ));

        runAction(new SeriesAction(
            Arrays.asList(
                new DriveForwardAndTurnToTarget(-20.0, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(angleToCargoThree)),
                new ExtendRetractBallIntake(true)
            )
        ));
    }
}