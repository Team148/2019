// package frc.auto.actions;

// import frc.robot.RobotState;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;
// import lib.geometry.Pose2dWithCurvature;
// import lib.trajectory.TimedView;
// import lib.trajectory.Trajectory;
// import lib.trajectory.TrajectoryIterator;
// import lib.trajectory.timing.TimedState;
// import edu.wpi.first.wpilibj.Timer;
// import frc.paths.TrajectoryGenerator;

// public class RocketTrajectoryWithOffset implements Action {
//     private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
//     private static final Drivetrain mDrive = Drivetrain.getInstance();
//     private static final RobotState mRobotState = RobotState.getInstance();
//     private static final Limelight mLL = Limelight.getInstance();
//     private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
//     private final boolean mResetPose;
//     private double offset;

//     public RocketTrajectoryWithOffset(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
//         this(trajectory, false);
//     }


//     public RocketTrajectoryWithOffset(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
//         mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
//         mResetPose = resetPose;
//     }

//     @Override
//     public boolean isFinished() {
//         if (mDrive.isDoneWithTrajectory()) {
//             System.out.println("Trajectory finished");
//             return true;
//         }
//         return false;
//     }

//     @Override
//     public void update() {
//     }

//     @Override
//     public void done() {
//     }

//     @Override
//     public void start() {
//         if (mResetPose) {
//             mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
//         }

//         offset = mLL.GetOffsetAngle();

//         if(offset > 1.64) {
//             mDrive.setTrajectory(mTrajectory);
//             if(offset > 3.07) {
//                 if(offset > 4.71) {
//                     if(offset > 6.13) {
//                         if(offset > 7.64){

//                         }
//                     }
//                 }
//             }
//         }
//         else if(offset < -1.64) {
//             if(offset < -3.07) {
//                 if(offset < -4.71) {
//                     if(offset < -6.13) {
//                         if(offset < 7.64) {

//                         }
//                     }
//                 }
//             }
//         }
//         else {

//         }
        
        
//     }
// }