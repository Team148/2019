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
    private double offset;

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToRocketThree;
    private DriveTrajectory mRocketThree;
    private DriveTrajectory mRocketOneOffsetCorrectPlus1;
    private DriveTrajectory mRocketOneOffsetCorrectPlus2;
    private DriveTrajectory mRocketOneOffsetCorrectPlus3;
    private DriveTrajectory mRocketOneOffsetCorrectPlus4;
    private DriveTrajectory mRocketOneOffsetCorrectPlus5;
    private DriveTrajectory mRocketOneOffsetCorrectMinus1;
    private DriveTrajectory mRocketOneOffsetCorrectMinus2;
    private DriveTrajectory mRocketOneOffsetCorrectMinus3;
    private DriveTrajectory mRocketOneOffsetCorrectMinus4;
    private DriveTrajectory mRocketOneOffsetCorrectMinus5;
    private DriveTrajectory mEndRocketToTurn;
    private DriveTrajectory mEndTurnToLoadingStation;
    private DriveTrajectory mLoadingStationToRocketTwo;

    public RocketModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        if (mStartedLeft) {
            mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().leftLevel1toRocketThree, true);
        } else {
            // mLevel1ToRocketOne = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightLevel1toRocketOne, true);
            mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rightLevel1toRocketThree, true);
        }

        // mRocketThree = new RocketTrajectoryWithOffset(mTrajectoryGenerator.getTrajectorySet().rocketThree.get(mStartedLeft));

        mRocketOneOffsetCorrectPlus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionPlus1.get(mStartedLeft));
        mRocketOneOffsetCorrectPlus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionPlus2.get(mStartedLeft));
        mRocketOneOffsetCorrectPlus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionPlus3.get(mStartedLeft));
        mRocketOneOffsetCorrectPlus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionPlus4.get(mStartedLeft));
        mRocketOneOffsetCorrectPlus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionPlus5.get(mStartedLeft));
        mRocketOneOffsetCorrectMinus1 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionMinus1.get(mStartedLeft));
        mRocketOneOffsetCorrectMinus2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionMinus2.get(mStartedLeft));
        mRocketOneOffsetCorrectMinus3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionMinus3.get(mStartedLeft));
        mRocketOneOffsetCorrectMinus4 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionMinus4.get(mStartedLeft));
        mRocketOneOffsetCorrectMinus5 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().rocketThreeWithCorrectionMinus5.get(mStartedLeft));
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
                mLevel1ToRocketThree
            )
        ));

        System.out.println("Finished Rocket Three Path");
        offset = LL.GetOffsetAngle();

        if(offset > 1.64) {
            System.out.println("Stepped into the if statement!");
            runAction(new SeriesAction (
            Arrays.asList(
                mRocketOneOffsetCorrectPlus1
            )
        ));
            if(offset > 3.07) {
                runAction(new SeriesAction (
                Arrays.asList(
                    // mRocketOneOffsetCorrectMinus2
                )
            ));
                if(offset > 4.71) {
                    runAction(new SeriesAction (
                    Arrays.asList(
                        // mRocketOneOffsetCorrectMinus3
                    )
                    )); 
                    if(offset > 6.13) {
                        runAction(new SeriesAction (
                        Arrays.asList(
                            // mRocketOneOffsetCorrectMinus4
                        )
                    ));
                        if(offset > 7.64){
                            runAction(new SeriesAction (
                            Arrays.asList(
                                // mRocketOneOffsetCorrectMinus5
                            )
                        ));
                        }
                    }
                }
            }
        }
        else if(offset < -1.64) {
            runAction(new SeriesAction (
            Arrays.asList(
                // mRocketOneOffsetCorrectPlus1
            )
        ));
            if(offset < -3.07) {
                runAction(new SeriesAction (
                    Arrays.asList(
                        // mRocketOneOffsetCorrectPlus2
                    )
                ));
                if(offset < -4.71) {
                    runAction(new SeriesAction (
                        Arrays.asList(
                            // mRocketOneOffsetCorrectPlus3
                        )
                    ));
                    if(offset < -6.13) {
                        runAction(new SeriesAction (
                            Arrays.asList(
                                // mRocketOneOffsetCorrectPlus4
                            )
                        ));
                        if(offset < 7.64) {
                            runAction(new SeriesAction (
                                Arrays.asList(
                                    // mRocketOneOffsetCorrectPlus5
                                )
                            ));
                        }
                    }
                }
            }
        }
        else {
            
        }

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