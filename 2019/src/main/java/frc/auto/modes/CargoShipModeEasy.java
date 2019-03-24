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
    private DriveTrajectory mLoadingStationNoCorrection;

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
    private DriveTrajectory mCargoTwoAway;
    private DriveTrajectory mToLoadingStation;
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
    private DriveTrajectory mLoadingStationToCargoThreeLineup;

    public CargoShipModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToCargoTwoLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineup.get(mStartedLeft), true);

        mCargoTwoNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToCargoTwo.get(mStartedLeft));
        // mRocketThreeNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeLineupToRocketThree.get(mStartedLeft));
        mLoadingStationNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationLineupToLoadingStation.get(mStartedLeft));

        mCargoTwoCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusOne.get(mStartedLeft));
        mCargoTwoCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusTwo.get(mStartedLeft));
        mCargoTwoCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusThree.get(mStartedLeft));
        mCargoTwoCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusFour.get(mStartedLeft));
        mCargoTwoCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionPlusFive.get(mStartedLeft));
        mCargoTwoCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusOne.get(mStartedLeft));
        mCargoTwoCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusTwo.get(mStartedLeft));
        mCargoTwoCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusThree.get(mStartedLeft));
        mCargoTwoCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusFour.get(mStartedLeft));
        mCargoTwoCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoCorrectionMinusFive.get(mStartedLeft));
        
        mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwo.get(mStartedLeft));
        mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToLoadingStation.get(mStartedLeft));

        mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineup.get(mStartedLeft));

        // mLoadingStationCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionPlusOne.get(mStartedLeft));
        // mLoadingStationCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionPlusTwo.get(mStartedLeft));
        // mLoadingStationCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionPlusThree.get(mStartedLeft));
        // mLoadingStationCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionPlusFour.get(mStartedLeft));
        // mLoadingStationCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionPlusFive.get(mStartedLeft));
        // mLoadingStationCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionMinusOne.get(mStartedLeft));
        // mLoadingStationCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionMinusTwo.get(mStartedLeft));
        // mLoadingStationCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionMinusThree.get(mStartedLeft));
        // mLoadingStationCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionMinusFour.get(mStartedLeft));
        // mLoadingStationCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationCorrectionMinusFive.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mLevel1ToCargoTwoLineup,
                // new TurnToHeading(Rotation2d.fromDegrees(180.0)),
                new WaitAction(0.25)
            )
        ));

        angleOffset = LL.GetOffsetAngle();
        inchOffset = (int) Math.floor(angleOffset / 1.3);
        if (mStartedLeft) {
            inchOffset *= -1;
        }

        System.out.println("I AM " + inchOffset + " INCHES OFF!!!!!!!!!!!");

        if (inchOffset == 1) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectPlus1
                )
            ));
        }
        else if (inchOffset == 2) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectPlus2
                )
            ));
        }
        else if (inchOffset == 3) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectPlus3
                )
            ));
        }
        else if (inchOffset == 4) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectPlus4
                )
            ));
        }
        else if (inchOffset >= 5) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectPlus5
                )
            ));
        }
        else if (inchOffset == -1) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectMinus1
                )
            ));
        }
        else if (inchOffset == -2) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectMinus2
                )
            ));
        }
        else if (inchOffset == -3) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectMinus3
                )
            ));
        }
        else if (inchOffset == -4) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectMinus4
                )
            ));
        }
        else if (inchOffset <= -5) {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoCorrectMinus5
                )
            ));
        }
        else {
            runAction(new SeriesAction (
                Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mCargoTwoNoCorrection
                )
            ));
        }

        runAction(new SeriesAction (
            Arrays.asList(
                // new OpenLoopDrive(0.3, 0.3, 0.3),
                new OpenCloseBeak(true)
            )
        ));

        // //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mCargoTwoAway,
                mToLoadingStation,
                new OpenCloseBeak(false),
                mLoadingStationToCargoThreeLineup
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