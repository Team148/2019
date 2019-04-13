package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
// import lib.geometry.Translation2d;

import java.util.Arrays;

import javax.swing.text.html.ParagraphView;

public class RocketModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToRocketOneLineup;

    private DriveTrajectory mRocketOneAway;
    private DriveTrajectory mToLoadingStation;

    private DriveTrajectory mLoadingStationToRocketThreeLineup;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        if(mStartedLeft) {
            mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineupLeft, true);

            mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneLeft);
            mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineupLeft);

            mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineupLeft);
        }
        else {
            mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineupRight, true);

            mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneRight);
            mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineupRight);

            mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineupRight, true);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        // Score First Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         new ExtendRetract4Bar(true),
        //         mLevel1ToRocketOneLineup
        //     )
        // ));

        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel1ToRocketOneLineup
                )),
                new DriveForwardAndTurnToTarget(20.0, 1.25),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.5),
                new TurnToHeading(Rotation2d.fromDegrees(180.0))
            )
        ));

        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mRocketOneAway
        //     )
        // ));

        // //Get Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
        //         mToLoadingStation
        //     )
        // ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(80.0, 1.5),
                new DriveForwardAndTurnToTarget(20.0, 1.0),
                new OpenCloseBeak(false),
                new OpenLoopDrive(-0.15, -0.15, 0.5)
            )
        ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         mLoadingStationToRocketThreeLineup
        //     )
        // ));
    }
}