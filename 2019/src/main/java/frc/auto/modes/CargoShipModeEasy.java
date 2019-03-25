package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import frc.robot.subsystems.Limelight;

import java.util.Arrays;

public class CargoShipModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private static final Limelight LL = Limelight.getInstance();
    private double angleOffset;
    private int inchOffset;

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToCargoTwoLineup;
    private DriveTrajectory mCargoTwoNoCorrection;
    // private DriveTrajectory mRocketThreeNoCorrection;
    private DriveTrajectory mCargoThreeNoCorrection;

    private DriveTrajectory mCargoTwoCorrectPlus1;
    private DriveTrajectory mCargoTwoCorrectPlus2;
    private DriveTrajectory mCargoTwoCorrectPlus3;
    private DriveTrajectory mCargoTwoCorrectPlus4;
    private DriveTrajectory mCargoTwoCorrectPlus5;
    private DriveTrajectory mCargoTwoCorrectMinus1;
    private DriveTrajectory mCargoTwoCorrectMinus2;
    private DriveTrajectory mCargoTwoCorrectMinus3;
    private DriveTrajectory mCargoTwoCorrectMinus4;
    private DriveTrajectory mCargoTwoCorrectMinus5;
    private DriveTrajectory mCargoTwoAway;
    private DriveTrajectory mToLoadingStation;
    private DriveTrajectory mLoadingStationToCargoThreeLineup;
    private DriveTrajectory mCargoThreeCorrectPlus1;
    private DriveTrajectory mCargoThreeCorrectPlus2;
    private DriveTrajectory mCargoThreeCorrectPlus3;
    private DriveTrajectory mCargoThreeCorrectPlus4;
    private DriveTrajectory mCargoThreeCorrectPlus5;
    private DriveTrajectory mCargoThreeCorrectMinus1;
    private DriveTrajectory mCargoThreeCorrectMinus2;
    private DriveTrajectory mCargoThreeCorrectMinus3;
    private DriveTrajectory mCargoThreeCorrectMinus4;
    private DriveTrajectory mCargoThreeCorrectMinus5;

    public CargoShipModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToCargoTwoLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineup.get(mStartedLeft), true);

        mCargoTwoNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToCargoTwo.get(mStartedLeft));
        // mRocketThreeNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeLineupToRocketThree.get(mStartedLeft));
        mCargoThreeNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeLineupToCargoThree.get(mStartedLeft));

        // mCargoTwoCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusOne.get(mStartedLeft));
        // mCargoTwoCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusTwo.get(mStartedLeft));
        // mCargoTwoCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusThree.get(mStartedLeft));
        // mCargoTwoCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusFour.get(mStartedLeft));
        // mCargoTwoCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusFive.get(mStartedLeft));
        // mCargoTwoCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusOne.get(mStartedLeft));
        // mCargoTwoCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusTwo.get(mStartedLeft));
        // mCargoTwoCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusThree.get(mStartedLeft));
        // mCargoTwoCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusFour.get(mStartedLeft));
        // mCargoTwoCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusFive.get(mStartedLeft));
        
        mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwo.get(mStartedLeft));
        mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToLoadingStation.get(mStartedLeft),true);

        mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineup.get(mStartedLeft), true);

        // mCargoThreeCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionPlusOne.get(mStartedLeft));
        // mCargoThreeCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionPlusTwo.get(mStartedLeft));
        // mCargoThreeCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionPlusThree.get(mStartedLeft));
        // mCargoThreeCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionPlusFour.get(mStartedLeft));
        // mCargoThreeCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionPlusFive.get(mStartedLeft));
        // mCargoThreeCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionMinusOne.get(mStartedLeft));
        // mCargoThreeCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionMinusTwo.get(mStartedLeft));
        // mCargoThreeCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionMinusThree.get(mStartedLeft));
        // mCargoThreeCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionMinusFour.get(mStartedLeft));
        // mCargoThreeCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoThreeCorrectionMinusFive.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                new ExtendRetract4Bar(true),
                mLevel1ToCargoTwoLineup
                // new TurnToHeading(Rotation2d.fromDegrees(180.0)),
                // new WaitAction(0.25)
            )
        ));

        // angleOffset = LL.GetOffsetAngle();
        // inchOffset = (int) Math.floor(angleOffset / 1.3);
        // if (mStartedLeft) {
        //     inchOffset *= -1;
        // }

        // System.out.println("I AM " + inchOffset + " INCHES OFF!!!!!!!!!!!");

        // if (inchOffset == 1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectPlus1
        //         )
        //     ));
        // }
        // else if (inchOffset == 2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectPlus2
        //         )
        //     ));
        // }
        // else if (inchOffset == 3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectPlus3
        //         )
        //     ));
        // }
        // else if (inchOffset == 4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectPlus4
        //         )
        //     ));
        // }
        // else if (inchOffset >= 5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectPlus5
        //         )
        //     ));
        // }
        // else if (inchOffset == -1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectMinus1
        //         )
        //     ));
        // }
        // else if (inchOffset == -2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectMinus2
        //         )
        //     ));
        // }
        // else if (inchOffset == -3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectMinus3
        //         )
        //     ));
        // }
        // else if (inchOffset == -4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectMinus4
        //         )
        //     ));
        // }
        // else if (inchOffset <= -5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoCorrectMinus5
        //         )
        //     ));
        // }
        // else {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             new ExtendRetract4Bar(true),
        //             mCargoTwoNoCorrection
        //         )
        //     ));
        // }

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(0.3, 1.0),
                new OpenCloseBeak(true)
            )
        ));

        // //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mCargoTwoAway,
                mToLoadingStation,
                new DriveForwardAndTurnToTarget(0.3, 1.0),
                new OpenCloseBeak(false),
                mLoadingStationToCargoThreeLineup
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(0.3, 1.0),
                new OpenCloseBeak(true),
                new OpenLoopDrive(-0.2, -0.2, 0.5)
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
        //             mCargoThreeCorrectPlus1
        //         )
        //     ));
        // }
        // else if (inchOffset == 2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectPlus2
        //         )
        //     ));
        // }
        // else if (inchOffset == 3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectPlus3
        //         )
        //     ));
        // }
        // else if (inchOffset == 4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectPlus4
        //         )
        //     ));
        // }
        // else if (inchOffset >= 5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectPlus5
        //         )
        //     ));
        // }
        // else if (inchOffset == -1) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectMinus1
        //         )
        //     ));
        // }
        // else if (inchOffset == -2) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectMinus2
        //         )
        //     ));
        // }
        // else if (inchOffset == -3) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectMinus3
        //         )
        //     ));
        // }
        // else if (inchOffset == -4) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectMinus4
        //         )
        //     ));
        // }
        // else if (inchOffset <= -5) {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeCorrectMinus5
        //         )
        //     ));
        // }
        // else {
        //     runAction(new SeriesAction (
        //         Arrays.asList(
        //             mCargoThreeNoCorrection
        //         )
        //     ));
        // }
    }
}