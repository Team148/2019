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

    private DriveTrajectory mGoStraight;

    // private DriveTrajectory mLevel1ToCargoTwoLineup;
    private DriveTrajectory mLevel1ToCargoTwoLineupForward;
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
    private DriveTrajectory mEndCargoTwoToLoadingStation;
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

        mGoStraight = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().goStraight.get(mStartedLeft), true);

        // mLevel1ToCargoTwoLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineup.get(mStartedLeft), true);
        mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForward.get(mStartedLeft), true);

        mCargoTwoNoCorrection = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToCargoTwo.get(mStartedLeft));
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
        mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStaton.get(mStartedLeft));

        mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToLoadingStation.get(mStartedLeft));

        mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineup.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                // mGoStraight
                // new ExtendRetract4Bar(true),
                // mLevel1ToCargoTwoLineup
                mLevel1ToCargoTwoLineupForward
                // new TurnToHeading(Rotation2d.fromDegrees(180.0)),
                // new WaitAction(0.25)
            )
        ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new DriveForwardAndTurnToTarget(0.2, 1.0),
        //         new OpenCloseBeak(true)
        //     )
        // ));

        // // //Get Second Hatch
        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         mCargoTwoAway,
        //         mEndCargoTwoToLoadingStation,
        //         // mToLoadingStation,
        //         new DriveForwardAndTurnToTarget(0.2, 1.0),
        //         new OpenCloseBeak(false),
        //         mLoadingStationToCargoThreeLineup
        //     )
        // ));

        // runAction(new SeriesAction (
        //     Arrays.asList(
        //         new DriveForwardAndTurnToTarget(0.2, 1.0),
        //         new OpenCloseBeak(true),
        //         new OpenLoopDrive(-0.2, -0.2, 0.5)
        //     )
        // ));
    }
}