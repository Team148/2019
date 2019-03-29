package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import frc.robot.subsystems.Limelight;

import java.util.Arrays;

public class RocketModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private static final Limelight LL = Limelight.getInstance();
    private double angleOffset;
    private int inchOffset;

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToRocketOneLineup;

    private DriveTrajectory mRocketOneAway;
    private DriveTrajectory mToLoadingStation;

    private DriveTrajectory mLoadingStationToRocketThreeLineup;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineup.get(mStartedLeft), true);

        mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOne.get(mStartedLeft));
        mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineup.get(mStartedLeft));

        mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineup.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                new ExtendRetract4Bar(true),
                mLevel1ToRocketOneLineup
                // new TurnToHeading(Rotation2d.fromDegrees(180.0)),
                // new WaitAction(0.25)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
            )
        ));

        runAction(new ParallelAction (
            Arrays.asList(
                mRocketOneAway
                // new ExtendRetractBallIntake(true)
            )
        ));

        // //Get Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                new ExtendRetract4Bar(false),
                mToLoadingStation
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new ExtendRetract4Bar(true),
                new OpenLoopDrive(-0.1, -0.1, 0.2),
                new OpenCloseBeak(false)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                mLoadingStationToRocketThreeLineup
                // new OpenLoopDrive(0.3, 0.3, 0.5)
                // new DriveForwardAndTurnToTarget(0.3, 1.0),
                // new OpenCloseBeak(true)
            )
        ));
    }
}