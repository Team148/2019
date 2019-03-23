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
    private DriveTrajectory mLevel1ToRocketThreeLineup;
    private DriveTrajectory mRocketThreeNoCorrection;
    private DriveTrajectory mRocketThreeCorrectPlus1;
    private DriveTrajectory mRocketThreeCorrectPlus2;
    private DriveTrajectory mRocketThreeCorrectPlus3;
    private DriveTrajectory mRocketThreeCorrectPlus4;
    private DriveTrajectory mRocketThreeCorrectPlus5;
    private DriveTrajectory mRocketThreeCorrectMinus1;
    private DriveTrajectory mRocketThreeCorrectMinus2;
    private DriveTrajectory mRocketThreeCorrectMinus3;
    private DriveTrajectory mRocketThreeCorrectMinus4;
    private DriveTrajectory mRocketThreeCorrectMinus5;
    private DriveTrajectory mEndRocketToTurn;
    private DriveTrajectory mEndTurnToLoadingStation;
    private DriveTrajectory mLoadingStationToRocketTwo;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToRocketThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToRocketThreeLineup.get(mStartedLeft), true);

        mRocketThreeNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeLineupToRocketThree.get(mStartedLeft));

        mRocketThreeCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionPlusOne.get(mStartedLeft));
        mRocketThreeCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionPlusTwo.get(mStartedLeft));
        mRocketThreeCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionPlusThree.get(mStartedLeft));
        mRocketThreeCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionPlusFour.get(mStartedLeft));
        mRocketThreeCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionPlusFive.get(mStartedLeft));
        mRocketThreeCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionMinusOne.get(mStartedLeft));
        mRocketThreeCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionMinusTwo.get(mStartedLeft));
        mRocketThreeCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionMinusThree.get(mStartedLeft));
        mRocketThreeCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionMinusFour.get(mStartedLeft));
        mRocketThreeCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeCorrectionMinusFive.get(mStartedLeft));
        // mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToRocketThree.get(mStartedLeft), true);
        // mLevel1ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToRocketOne.get(mStartedLeft), true);
        // mEndRocketToTurn = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketToTurn.get(mStartedLeft));
        // mEndTurnToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endRocketTurnToLoadingStation.get(mStartedLeft));
        // mLoadingStationToRocketTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToRocketTwo.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mLevel1ToRocketThreeLineup,
                new TurnToHeading(Rotation2d.fromDegrees(210.0)),
                new WaitAction(0.25)
            )
        ));

        // angleOffset = LL.GetOffsetAngle();
        // inchOffset = (int) Math.floor(angleOffset / 1.3);
        // if (mStartedLeft) {
        //     inchOffset *= -1;
        // }

        // if (inchOffset == 1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectPlus1
        //         )
        //     ));
        // }
        // else if (inchOffset == 2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectPlus2
        //         )
        //     ));
        // }
        // else if (inchOffset == 3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectPlus3
        //         )
        //     ));
        // }
        // else if (inchOffset == 4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectPlus4
        //         )
        //     ));
        // }
        // else if (inchOffset >= 5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectPlus5
        //         )
        //     ));
        // }
        // else if (inchOffset == -1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectMinus1
        //         )
        //     ));
        // }
        // else if (inchOffset == -2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectMinus2
        //         )
        //     ));
        // }
        // else if (inchOffset == -3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectMinus3
        //         )
        //     ));
        // }
        // else if (inchOffset == -4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectMinus4
        //         )
        //     ));
        // }
        // else if (inchOffset <= -5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeCorrectMinus5
        //         )
        //     ));
        // }
        // else {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mRocketThreeNoCorrection
        //         )
        //     ));
        // }

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new OpenLoopDrive(0.3, 0.3, 0.3),
        //         new OpenCloseBeak(true),
        //         new WaitAction(0.25)
        //     )
        // ));

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
        //         new OpenLoopDrive(0.5, 0.5, 0.2)
        //     )
        // ));
    }
}