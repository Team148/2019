package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class RocketModeHard extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel2ToRocketThree;
    private DriveTrajectory mEndRocketToTurn;
    private DriveTrajectory mEndTurnToLoadingStation;
    private DriveTrajectory mLoadingStationToRocketTwo;

    public RocketModeHard(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel2ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelTwoToRocketThreeLineup.get(mStartedLeft), true);

        // mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToRocketThree.get(mStartedLeft), true);
        // mLevel2ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level2ToRocketOne.get(mStartedLeft), true);
        // mEndRocketToTurn = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketToTurn.get(mStartedLeft));
        // mEndTurnToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketTurnToLoadingStation.get(mStartedLeft));
        // mLoadingStationToRocketTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketTwo.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        // //Score First Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mLevel2ToRocketOne
        //     )
        // ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new TurnToTarget(0.25),
        //         new WaitAction(1.0)
        //         // new ExtendRetract4Bar(true),
        //         // new OpenLoopDrive(0.5, 0.5, 0.5)
        //     )
        // ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new OpenCloseBeak(true),
        //         new WaitAction(0.25),
        //         new ExtendRetractBallIntake(true),
        //         new WaitAction(0.5)
        //     )
        // ));

        // //Get Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mEndRocketToTurn,
        //         new ExtendRetractBallIntake(false)
        //     )
        // ));

        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mEndTurnToLoadingStation
        //     )
        // ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new OpenCloseBeak(false),
        //         new WaitAction(0.25),
        //         new OpenLoopDrive(-0.5, -0.5, 2.0)

        //     )
        // ));

        // // //Score Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mLoadingStationToRocketTwo
        //     )
        // ));

        // //Score Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         new OpenLoopDrive(0.5, 0.5, 0.2, false)
        //     )
        // ));
    }
}