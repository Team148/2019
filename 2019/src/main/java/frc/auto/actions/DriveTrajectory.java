package frc.auto.actions;

import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.TimedView;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrajectory implements Action {
    private static final Drivetrain mDrive = Drivetrain.getInstance();
    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;
    private final boolean mResetTransform;
    private final boolean mResetHeading;
    private final WaitUntilInsideRegion mWaitInsideRegion;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }


    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
        mResetTransform = false;
        mResetHeading = false;

        mWaitInsideRegion = null;
    }

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetTransform, boolean resetHeading) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = false;
        mResetTransform = resetTransform;
        mResetHeading = resetTransform;
         
        mWaitInsideRegion = null;
    }

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetTransform, boolean resetHeading, Translation2d bottomLeft, Translation2d topRight, boolean isOnLeft) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = false;
        mResetTransform = resetTransform;
        mResetHeading = resetTransform;

        mWaitInsideRegion = new WaitUntilInsideRegion(bottomLeft, topRight, isOnLeft);
    }

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            // System.out.println("Trajectory finished");
            System.out.println("Trajectory Finished at X: " + mRobotState.getLatestFieldToVehicle().getValue().getTranslation().x()
                                        + " Y: " + mRobotState.getLatestFieldToVehicle().getValue().getTranslation().y()
                                        + " Heading: " + mRobotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees()
                                        + " Vel: " +mRobotState.getMeasuredVelocity().dx);
            return true;
        }
        if(mWaitInsideRegion != null) {
            if (mWaitInsideRegion.isFinished()) {
                System.out.println("Inside box!");
                // System.out.println("Trajectory Finished at X: " + mRobotState.getLatestFieldToVehicle().getValue().getTranslation().x()
                //                         + " Y: " + mRobotState.getLatestFieldToVehicle().getValue().getTranslation().y()
                //                         + " Heading: " + mRobotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees()
                //                         + " Vel: " +mRobotState.getMeasuredVelocity().dx);
                return true;
            }
        }
        return false;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            System.out.println("RESETTING ROBOT POSE!!");
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }

        if(mResetHeading && mResetTransform)
        {
            System.out.println("RESETTING ROBOT POSE!!");
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        if(mResetTransform && !mResetHeading){
            System.out.println("RESETTING ROBOT TRANSLATION!!");
            Translation2d temp_translation = mTrajectory.getState().state().getPose().getTranslation();
            Rotation2d current_rotation = mDrive.getHeading();

            mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d(temp_translation, current_rotation));
        }
            
        mDrive.setTrajectory(mTrajectory);
    }
}

