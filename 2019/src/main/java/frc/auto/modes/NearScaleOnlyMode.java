package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class NearScaleOnlyMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private final boolean mStartedLeft;
//     private DriveTrajectory mSideStartToNearScale;
//     private DriveTrajectory mNearScaleToNearFence;
//     private DriveTrajectory mNearFenceToNearScale;
//     private DriveTrajectory mNearScaleToNearFence2;
//     private DriveTrajectory mNearFence2ToNearScale;
//     private DriveTrajectory mNearScaleToNearFence3;
//     private DriveTrajectory mNearFence3ToNearScale;
//     private DriveTrajectory mNearFence3ToEndPose;

//     private double mNearFenceWaitTime, mNearFence2WaitTime, mNearFence3WaitTime;

    public NearScaleOnlyMode(boolean robotStartedOnLeft) {
        mStartedLeft = robotStartedOnLeft;
        // mSideStartToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true);
        // mNearScaleToNearFence = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft));
        // mNearFenceToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFenceToNearScale.get(mStartedLeft));
        // mNearScaleToNearFence2 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft));
        // mNearFence2ToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence2ToNearScale.get(mStartedLeft));
        // mNearScaleToNearFence3 = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft));
        // mNearFence3ToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence3ToNearScale.get(mStartedLeft));
        // mNearFence3ToEndPose = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().nearFence3ToEndPose.get(mStartedLeft));

        // mNearFenceWaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft).getLastState().t() - 0.15 + (AutoConstants.kUseKickstand ? 0.25 : 0.0);
        // mNearFence2WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft).getLastState().t() - 0.15;
        // mNearFence3WaitTime = mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft).getLastState().t() - 0.15;

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // System.out.println("Running easy scale only");

//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mSideStartToNearScale,
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new WaitAction(1.0)
//                                 )
//                         ),
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new WaitUntilInsideRegion(new Translation2d(80.0, -20.0), new Translation2d
//                                                 (260, 50), mStartedLeft),

//                                         new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
//                                                 (260, 1000), mStartedLeft)
//                                 )
//                         )
//                 )
//         ));

//         // Get second cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         mNearScaleToNearFence
//                                 )
//                         ),
//                         new SeriesAction(Arrays.asList(
//                                 new WaitAction(mNearFenceWaitTime)
//                         ))
//                 )
//         ));

//         // Score second cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearFenceToNearScale,
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new ParallelAction(Arrays.asList(
//                                                 new WaitAction(AutoConstants.kWaitForCubeTime)
//                                         )),

//                                         new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
//                                                 (260, 1000), mStartedLeft)
//                                 )
//                         )
//                 )
//         ));

//         // Get third cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearScaleToNearFence2,
//                         new SeriesAction(Arrays.asList(
//                                 new WaitAction(mNearFence2WaitTime)
//                         ))
//                 )
//         ));
// //        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

//         // Score third cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearFence2ToNearScale,
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new ParallelAction(Arrays.asList(
//                                                 new WaitAction(AutoConstants.kWaitForCubeTime)
//                                         )),

//                                         new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
//                                                 (260, 1000), mStartedLeft)
//                                 )
//                         )
//                 )
//         ));

//         // Get fourth cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearScaleToNearFence3,
//                         new SeriesAction(Arrays.asList(
//                                 new WaitAction(mNearFence3WaitTime)
//                         ))
//                 )
//         ));
// //        runAction(new WaitAction(AutoConstants.kWaitForCubeTime));

//         /*
//         // Get fourth cube in position to score
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearFence3ToEndPose,
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new WaitAction(AutoConstants.kWaitForCubeTime),
//                                         new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight,
//                                                 SuperstructureConstants.kStowedPositionAngle, true)
//                                 )
//                         )
//                 )
//         ));*/

//         // Score fourth cube
//         runAction(new ParallelAction(
//                 Arrays.asList(
//                         mNearFence3ToNearScale,
//                         new SeriesAction(
//                                 Arrays.asList(
//                                         new ParallelAction(Arrays.asList(
//                                                 new WaitAction(AutoConstants.kWaitForCubeTime)
//                                         )),

//                                         new WaitUntilInsideRegion(new Translation2d(245.0, -1000.0), new Translation2d
//                                                 (260, 1000), mStartedLeft)
//                                 )
//                         )
//                 )
//         ));

    }
}
