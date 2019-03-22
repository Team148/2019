package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
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
                new WaitAction(0.25)
            )
        ));

        System.out.println("Finished Rocket Three Path");
        angleOffset = LL.GetOffsetAngle();
        inchOffset = (int) Math.floor(angleOffset / 1.3);
        if (mStartedLeft) {
            inchOffset *= -1;
        }

        if (inchOffset == 1) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectPlus1
                )
            ));
        }
        else if (inchOffset == 2) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectPlus2
                )
            ));
        }
        else if (inchOffset == 3) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectPlus3
                )
            ));
        }
        else if (inchOffset == 4) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectPlus4
                )
            ));
        }
        else if (inchOffset >= 5) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectPlus5
                )
            ));
        }
        else if (inchOffset == -1) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectMinus1
                )
            ));
        }
        else if (inchOffset == -2) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectMinus2
                )
            ));
        }
        else if (inchOffset == -3) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectMinus3
                )
            ));
        }
        else if (inchOffset == -4) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectMinus4
                )
            ));
        }
        else if (inchOffset <= -5) {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeCorrectMinus5
                )
            ));
        }
        else {
            runAction(new SeriesAction (
                Arrays.asList(
                    mRocketThreeNoCorrection
                )
            ));
        }

        // if(offset > 1.64) {
        //     System.out.println("Stepped into the if statement!");
        //     runAction(new SeriesAction (
        //     Arrays.asList(
        //         mRocketOneCorrectPlus1
        //     )
        // ));
        //     if(offset > 3.07) {
        //         runAction(new SeriesAction (
        //         Arrays.asList(
        //             // mRocketOneOffsetCorrectMinus2
        //         )
        //     ));
        //         if(offset > 4.71) {
        //             runAction(new SeriesAction (
        //             Arrays.asList(
        //                 // mRocketOneOffsetCorrectMinus3
        //             )
        //             )); 
        //             if(offset > 6.13) {
        //                 runAction(new SeriesAction (
        //                 Arrays.asList(
        //                     // mRocketOneOffsetCorrectMinus4
        //                 )
        //             ));
        //                 if(offset > 7.64){
        //                     runAction(new SeriesAction (
        //                     Arrays.asList(
        //                         // mRocketOneOffsetCorrectMinus5
        //                     )
        //                 ));
        //                 }
        //             }
        //         }
        //     }
        // }
        // else if(offset < -1.64) {
        //     runAction(new SeriesAction (
        //     Arrays.asList(
        //         // mRocketOneOffsetCorrectPlus1
        //     )
        // ));
        //     if(offset < -3.07) {
        //         runAction(new SeriesAction (
        //             Arrays.asList(
        //                 // mRocketOneOffsetCorrectPlus2
        //             )
        //         ));
        //         if(offset < -4.71) {
        //             runAction(new SeriesAction (
        //                 Arrays.asList(
        //                     // mRocketOneOffsetCorrectPlus3
        //                 )
        //             ));
        //             if(offset < -6.13) {
        //                 runAction(new SeriesAction (
        //                     Arrays.asList(
        //                         // mRocketOneOffsetCorrectPlus4
        //                     )
        //                 ));
        //                 if(offset < 7.64) {
        //                     runAction(new SeriesAction (
        //                         Arrays.asList(
        //                             // mRocketOneOffsetCorrectPlus5
        //                         )
        //                     ));
        //                 }
        //             }
        //         }
        //     }
        // }
        // else {
            
        // }

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