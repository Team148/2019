package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class RocketModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private double angleToLoadingStation;
    private double angleToRocketThree;

    private DriveTrajectory mLevel1ToRocketOneLineup;

    // private DriveTrajectory mRocketOneAway;
    // private DriveTrajectory mToLoadingStation;

    private DriveTrajectory mLoadingStationToRocketThreeLineup;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        if(mStartedLeft) {
            angleToLoadingStation = 180.0;
            angleToRocketThree = 150.0;

            mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineupLeft, true, false, TrajectoryGenerator.kRocketOneLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kRocketOneLineupPoseLeft.getTranslation().translateBy(new Translation2d(4.0, 4.0)), true);

            // mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneLeft);
            // mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineupLeft);

            mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineupLeft, false, false, TrajectoryGenerator.kRocketThreeLineupPoseLeft.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kRocketThreeLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), true);
        }
        else {
            angleToLoadingStation = 180.0;
            angleToRocketThree = 210.0;

            mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineupRight, true, false, TrajectoryGenerator.kRocketOneLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kRocketOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);

            // mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneRight);
            // mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineupRight);

            mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineupRight, false, false, TrajectoryGenerator.kRocketThreeLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kRocketThreeLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode Easy");

        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel1ToRocketOneLineup
                )),
                new DriveForwardAndTurnToTarget(20.0, 0.5),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(angleToLoadingStation), 0.75)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(80.0, 1.5),
                new DriveForwardAndTurnToTarget(20.0, 1.0),
                new OpenCloseBeak(false)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                mLoadingStationToRocketThreeLineup,
                new TurnToHeading(Rotation2d.fromDegrees(angleToRocketThree), 0.5),
                new DriveForwardAndTurnToTarget(20.0, 1.25),
                new OpenCloseBeak(true),
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(90.0))
            )
        ));
    }
}