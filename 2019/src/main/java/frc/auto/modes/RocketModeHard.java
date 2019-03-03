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
    private DriveTrajectory mLevel2ToRocketOne;
    private DriveTrajectory mEndRocketToTurn;
    private DriveTrajectory mEndTurnToLoadingStation;
    private DriveTrajectory mLoadingStationToRocketTwo;

    public RocketModeHard(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        if (mStartedLeft) {
            mLevel2ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().leftLevel2toRocketOne, true);
        } else {
            mLevel2ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightLevel2toRocketOne, true);
        }

        // mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToRocketThree.get(mStartedLeft), true);
        // mLevel2ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level2ToRocketOne.get(mStartedLeft), true);
        // mEndRocketToTurn = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketToTurn.get(mStartedLeft));
        // mEndTurnToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketTurnToLoadingStation.get(mStartedLeft));
        // mLoadingStationToRocketTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketTwo.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                mLevel2ToRocketOne
            )
        ));

        // // //Get Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mEndRocketToTurn
        //     )
        // ));

        // // //Score Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mEndTurnToLoadingStation
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