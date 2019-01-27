package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class SwitchAndScaleMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mSwitchLeft, mScaleLeft;
    private DriveTrajectory mStartToSwitch;
    private DriveTrajectory mSwitchToPyramidCube;
    private DriveTrajectory mPyramidCubeToScale;
    private DriveTrajectory mScaleToFence;
    private DriveTrajectory mFenceToScale;

    private double mStartCubeWaitTime, mFenceWaitTime, mPyramidWaitTime;

    public SwitchAndScaleMode(boolean isSwitchOnLeft, boolean isScaleOnLeft) {
        mSwitchLeft = isSwitchOnLeft;
        mScaleLeft = isScaleOnLeft;

        // if (isSwitchOnLeft) {
        //     mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch, true);
        //     mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToLeftSwitch.getLastState().t() - 0.1;
        // } else {
        //     mStartToSwitch = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch, true);
        //     mStartCubeWaitTime = mTrajectoryGenerator.getTrajectorySet().centerStartToRightSwitch.getLastState().t() - 0.1;
        // }
        // if (isScaleOnLeft) {
        //     mPyramidCubeToScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerPyramidCubeToScaleLeft.get(mScaleLeft));
        //     mScaleToFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().scaleToFenceLeft.get(mScaleLeft));
        //     mFenceToScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().fenceToScaleLeft.get(mScaleLeft));
        //     mFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().scaleToFenceLeft.get(mScaleLeft).getLastState().t() - 0.15 + 0.25;
        // } else {
        //     mPyramidCubeToScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().centerPyramidCubeToScaleRight.get(mScaleLeft));
        //     mScaleToFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().scaleToFenceRight.get(mScaleLeft));
        //     mFenceToScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().fenceToScaleRight.get(mScaleLeft));
        //     mFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().scaleToFenceRight.get(mScaleLeft).getLastState().t() - 0.15 + 0.25;
        // }

        // mSwitchToPyramidCube = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().switchToCenterPyramidCube.get(mSwitchLeft));
        // mPyramidWaitTime = mTrajectoryGenerator.getTrajectorySet().switchToCenterPyramidCube.get(mScaleLeft).getLastState().t() - 0.15;

    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running scale+switch");

        //Score first cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mStartToSwitch,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mStartCubeWaitTime)
                                )
                        )
                )
        ));

        // Get second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mSwitchToPyramidCube,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(-1000.0, -50.0), new Translation2d
                                                (1000.0, 50.0), mSwitchLeft)
                                )
                        ),
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(mPyramidWaitTime)
                                )
                        )

                )
        ));

        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        //Score second cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mPyramidCubeToScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(140.0, -1000.0), new Translation2d
                                                (300.0, 1000.0), mScaleLeft),

                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (300, 1000), mScaleLeft)
                                )
                        )
                )
        ));

        // Get third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitAction(0.5),
                                        mScaleToFence
                                )
                        ),
                        new SeriesAction(Arrays.asList(
                                new WaitAction(mFenceWaitTime)
                        ))
                )
        ));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));
        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

        // Score third cube
        runAction(new ParallelAction(
                Arrays.asList(
                        mFenceToScale,
                        new SeriesAction(
                                Arrays.asList(
                                        new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
                                                (280, 1000), mScaleLeft),
                                        new WaitAction(AutoConstants.kWaitForCubeTime)
                                )
                        )
                )
        ));
    }

}
