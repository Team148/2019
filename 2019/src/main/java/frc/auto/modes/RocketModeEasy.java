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
    private DriveTrajectory mRocketOneNoCorrection;
    private DriveTrajectory mRocketThreeNoCorrection;
    private DriveTrajectory mLoadingStationNoCorrection;

    private DriveTrajectory mRocketOneCorrectPlus1;
    private DriveTrajectory mRocketOneCorrectPlus2;
    private DriveTrajectory mRocketOneCorrectPlus3;
    private DriveTrajectory mRocketOneCorrectPlus4;
    private DriveTrajectory mRocketOneCorrectPlus5;
    private DriveTrajectory mRocketOneCorrectMinus1;
    private DriveTrajectory mRocketOneCorrectMinus2;
    private DriveTrajectory mRocketOneCorrectMinus3;
    private DriveTrajectory mRocketOneCorrectMinus4;
    private DriveTrajectory mRocketOneCorrectMinus5;
    // private DriveTrajectory mRocketThreeCorrectPlus1;
    // private DriveTrajectory mRocketThreeCorrectPlus2;
    // private DriveTrajectory mRocketThreeCorrectPlus3;
    // private DriveTrajectory mRocketThreeCorrectPlus4;
    // private DriveTrajectory mRocketThreeCorrectPlus5;
    // private DriveTrajectory mRocketThreeCorrectMinus1;
    // private DriveTrajectory mRocketThreeCorrectMinus2;
    // private DriveTrajectory mRocketThreeCorrectMinus3;
    // private DriveTrajectory mRocketThreeCorrectMinus4;
    // private DriveTrajectory mRocketThreeCorrectMinus5;
    private DriveTrajectory mRocketOneAway;
    private DriveTrajectory mToLoadingStation;
    private DriveTrajectory mLoadingStationToRocketThreeLineup;
    private DriveTrajectory mLoadingStationCorrectPlus1;
    private DriveTrajectory mLoadingStationCorrectPlus2;
    private DriveTrajectory mLoadingStationCorrectPlus3;
    private DriveTrajectory mLoadingStationCorrectPlus4;
    private DriveTrajectory mLoadingStationCorrectPlus5;
    private DriveTrajectory mLoadingStationCorrectMinus1;
    private DriveTrajectory mLoadingStationCorrectMinus2;
    private DriveTrajectory mLoadingStationCorrectMinus3;
    private DriveTrajectory mLoadingStationCorrectMinus4;
    private DriveTrajectory mLoadingStationCorrectMinus5;
    private DriveTrajectory mLoadingStationToRocketTwo;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToRocketOneLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketOneLineup.get(mStartedLeft), true);

        mRocketOneNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneLineupToRocketOne.get(mStartedLeft));

        mLoadingStationNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationLineupToLoadingStation.get(mStartedLeft));

        mRocketOneCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionPlusOne.get(mStartedLeft), true);
        mRocketOneCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionPlusTwo.get(mStartedLeft), true);
        mRocketOneCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionPlusThree.get(mStartedLeft), true);
        mRocketOneCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionPlusFour.get(mStartedLeft), true);
        mRocketOneCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionPlusFive.get(mStartedLeft), true);
        mRocketOneCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionMinusOne.get(mStartedLeft), true);
        mRocketOneCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionMinusTwo.get(mStartedLeft), true);
        mRocketOneCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionMinusThree.get(mStartedLeft), true);
        mRocketOneCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionMinusFour.get(mStartedLeft), true);
        mRocketOneCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketOneCorrectionMinusFive.get(mStartedLeft), true);

        mRocketOneAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOne.get(mStartedLeft));
        mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromRocketOneToLoadingStationLineup.get(mStartedLeft));
        mLoadingStationToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketThreeLineup.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                new ExtendRetract4Bar(true),
                mLevel1ToRocketOneLineup
                // new TurnToHeading(Rotation2d.fromDegrees(180.0)),
                // new WaitAction(0.25)
            )
        ));

        // angleOffset = LL.GetOffsetAngle();
        // inchOffset = (int) Math.floor(angleOffset / 1.64);
        // if (mStartedLeft) {
        //     inchOffset *= -1;
        // }

        // System.out.println("I AM " + inchOffset + " INCHES OFF!!!!!!!!!!!");

        // if (inchOffset == 1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectPlus1
        //         )
        //     ));
        // }
        // else if (inchOffset == 2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectPlus2
        //         )
        //     ));
        // }
        // else if (inchOffset == 3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectPlus3
        //         )
        //     ));
        // }
        // else if (inchOffset == 4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectPlus4
        //         )
        //     ));
        // }
        // else if (inchOffset >= 5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectPlus5
        //         )
        //     ));
        // }
        // else if (inchOffset == -1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectMinus1
        //         )
        //     ));
        // }
        // else if (inchOffset == -2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectMinus2
        //         )
        //     ));
        // }
        // else if (inchOffset == -3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectMinus3
        //         )
        //     ));
        // }
        // else if (inchOffset == -4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectMinus4
        //         )
        //     ));
        // }
        // else if (inchOffset <= -5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneCorrectMinus5
        //         )
        //     ));
        // }
        // else {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketOneNoCorrection
        //         )
        //     ));
        // }

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(0.3, 1.0),
                new OpenCloseBeak(true)
            )
        ));

        runAction(new ParallelAction (
            Arrays.asList(
                mRocketOneAway,
                new ExtendRetractBallIntake(true)
            )
        ));

        // //Get Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                new ExtendRetractBallIntake(false),
                mToLoadingStation
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(0.3, 0.5),
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

        // angleOffset = LL.GetOffsetAngle();
        // inchOffset = (int) Math.floor(angleOffset / 1.3);
        // System.out.println("INCH OFFSET   " + inchOffset + "!!!!!!!!!!!");
        // if (mStartedLeft) {
        //     inchOffset *= -1;
        // }

        // if (inchOffset == 1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectMinus1
        //         )
        //     ));
        // }
        // else if (inchOffset == 2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectMinus2
        //         )
        //     ));
        // }
        // else if (inchOffset == 3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectMinus3
        //         )
        //     ));
        // }
        // else if (inchOffset == 4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectMinus4
        //         )
        //     ));
        // }
        // else if (inchOffset >= 5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectMinus5
        //         )
        //     ));
        // }
        // else if (inchOffset == -1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectPlus1
        //         )
        //     ));
        // }
        // else if (inchOffset == -2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectPlus2
        //         )
        //     ));
        // }
        // else if (inchOffset == -3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectPlus3
        //         )
        //     ));
        // }
        // else if (inchOffset == -4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectPlus4
        //         )
        //     ));
        // }
        // else if (inchOffset <= -5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mLoadingStationCorrectPlus5
        //         )
        //     ));
        // }
        // else {
            // runAction(new SeriesAction (
            //     Arrays.asList(
            //         mLoadingStationNoCorrection,
            //         new OpenCloseBeak(false),
            //         new OpenLoopDrive(-0.3, -0.3, -0.3)
            //     )
            // ));
        // }
    }
}