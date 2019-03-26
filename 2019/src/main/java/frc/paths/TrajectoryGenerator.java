package frc.paths;

import frc.planners.DriveMotionPlanner;
import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryUtil;
import lib.trajectory.timing.CentripetalAccelerationConstraint;
import lib.trajectory.timing.TimedState;
import lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.springframework.beans.factory.support.RootBeanDefinition;

public class TrajectoryGenerator {
    // private static final double kMaxVelocity = 50.0;
    // private static final double kMaxAccel = 50.0;
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 100.0;
    // private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    // private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxCentripetalAccelElevatorDown = 30.0;
    // private static final double kMaxCentripetalAccel = 50.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 11.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 130.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static final double kSimpleSwitchMaxAccel = 100.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    //Constant Poses
    //Start
    //Level 1
    public static final Pose2d kLevelOneStartPose = new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevelOneStartBackwardPose = new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(180.0));

    // //Level 2
    public static final Pose2d kLevelTwoStartPose = new Pose2d(new Translation2d(30.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevelTwoStartBackwardPose = new Pose2d(new Translation2d(30.0, -50.0), Rotation2d.fromDegrees(180.0));
 
    // //Off Level 2
    public static final Pose2d kOffHabPlatform = new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kOffHabPlatformBackwards = new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(180.0));

    // //Scoring Poses
    // //Cargo One
    public static final Pose2d kCargoOnePose = new Pose2d(new Translation2d(205.0, -11.0), Rotation2d.fromDegrees(0.0));

    // // Opposite Side Cargo One
    public static final Pose2d kOppositeCargoOnePose = new Pose2d(new Translation2d(205.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two
    public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(253.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Three
    public static final Pose2d kCargoThreePose = new Pose2d(new Translation2d(281.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Four
    public static final Pose2d kCargoFourPose = new Pose2d(new Translation2d(303.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Rocket One
    // public static final Pose2d kRocketOnePose = new Pose2d(new Translation2d(198.0, -135.0), Rotation2d.fromDegrees(330.0));
    public static final Pose2d kRocketOnePose = new Pose2d(new Translation2d(197.0, -133.0), Rotation2d.fromDegrees(330.0));

    // //Rocket Three
    public static final Pose2d kRocketThreePose = new Pose2d(new Translation2d(260.0, -135.0), Rotation2d.fromDegrees(210.0));

    // //Loading Station Poses
    public static final Pose2d kLoadingStationPose = new Pose2d(new Translation2d(23.0, -135.0), Rotation2d.fromDegrees(180.0)); 

    // //Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kCargoOneLineupPose = new Pose2d(new Translation2d(175.0, -11.0), Rotation2d.fromDegrees(0.0));

    // //Opposite Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kOppositeCargoOneLineupPose = new Pose2d(new Translation2d(175.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two Lineup (for Vision Tracking)
    public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(253.0, -80.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoBackupPose = new Pose2d(new Translation2d(253.0, -100.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Three Lineup (for Vision Tracking)
    public static final Pose2d kCargoThreeLineupPose = new Pose2d(new Translation2d(280.0, -80.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Four Lineup (for Vision Tracking)
    public static final Pose2d kCargoFourLineupPose = new Pose2d(new Translation2d(303.0, -70.0), Rotation2d.fromDegrees(90.0));

    // //Rocket One Lineup (for Vision Tracking)
    public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(171.0, -119.0), Rotation2d.fromDegrees(330.0));
    // public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(178.0, -125.0), Rotation2d.fromDegrees(330.0));

    // //Rocket Three Lineup (for Vision Tracking)
    public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -119.0), Rotation2d.fromDegrees(210.0));
    // public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -119.0), Rotation2d.fromDegrees(180.0));

    // //Loading Station Lineup (for Vision Tracking)
    public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(40.0, -135.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kComingToLoadingStation = kLoadingStationLineupPose.transformBy(Pose2d.fromTranslation(new Translation2d(30.0, 0.0)));

    // //Correction Poses (Minus 1 is 1 inch left, Plus 1 is 1 inch right)
    //Cargo Corrections
    public static final Pose2d kCargoOnePlusOne = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 1.0)));
    public static final Pose2d kCargoOnePlusTwo = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 2.0)));
    public static final Pose2d kCargoOnePlusThree = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 3.0)));
    public static final Pose2d kCargoOnePlusFour = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0)));
    public static final Pose2d kCargoOnePlusFive = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 5.0)));

    public static final Pose2d kCargoOneMinusOne = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -1.0)));
    public static final Pose2d kCargoOneMinusTwo = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -2.0)));
    public static final Pose2d kCargoOneMinusThree = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -3.0)));
    public static final Pose2d kCargoOneMinusFour = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -4.0)));
    public static final Pose2d kCargoOneMinusFive = kCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -5.0)));

    public static final Pose2d kOppositeCargoOnePlusOne = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -1.0)));
    public static final Pose2d kOppositeCargoOnePlusTwo = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -2.0)));
    public static final Pose2d kOppositeCargoOnePlusThree = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -3.0)));
    public static final Pose2d kOppositeCargoOnePlusFour = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -4.0)));
    public static final Pose2d kOppositeCargoOnePlusFive = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -5.0)));

    public static final Pose2d kOppositeCargoOneMinusOne = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 1.0)));
    public static final Pose2d kOppositeCargoOneMinusTwo = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 2.0)));
    public static final Pose2d kOppositeCargoOneMinusThree = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 3.0)));
    public static final Pose2d kOppositeCargoOneMinusFour = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0)));
    public static final Pose2d kOppositeCargoOneMinusFive = kOppositeCargoOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 5.0)));

    public static final Pose2d kCargoTwoPlusOne = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(1.0, 0.0)));
    public static final Pose2d kCargoTwoPlusTwo = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, 0.0)));
    public static final Pose2d kCargoTwoPlusThree = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(3.0, 0.0)));
    public static final Pose2d kCargoTwoPlusFour = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(4.0, 0.0)));
    public static final Pose2d kCargoTwoPlusFive = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(5.0, 0.0)));

    public static final Pose2d kCargoTwoMinusOne = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.0, 0.0)));
    public static final Pose2d kCargoTwoMinusTwo = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.0, 0.0)));
    public static final Pose2d kCargoTwoMinusThree = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(-3.0, 0.0)));
    public static final Pose2d kCargoTwoMinusFour = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(-4.0, 0.0)));
    public static final Pose2d kCargoTwoMinusFive = kCargoTwoPose.transformBy(Pose2d.fromTranslation(new Translation2d(-5.0, 0.0)));


    public static final Pose2d kCargoThreePlusOne = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.0, 0.0)));
    public static final Pose2d kCargoThreePlusTwo = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, 0.0)));
    public static final Pose2d kCargoThreePlusThree = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(3.0, 0.0)));
    public static final Pose2d kCargoThreePlusFour = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(4.0, 0.0)));
    public static final Pose2d kCargoThreePlusFive = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(5.0, 0.0)));

    public static final Pose2d kCargoThreeMinusOne = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.0, 0.0)));
    public static final Pose2d kCargoThreeMinusTwo = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.0, 0.0)));
    public static final Pose2d kCargoThreeMinusThree = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-3.0, 0.0)));
    public static final Pose2d kCargoThreeMinusFour = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-4.0, 0.0)));
    public static final Pose2d kCargoThreeMinusFive = kCargoThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-5.0, 0.0)));

    public static final Pose2d kCargoFourPlusOne = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(1.0, 0.0)));
    public static final Pose2d kCargoFourPlusTwo = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, 0.0)));
    public static final Pose2d kCargoFourPlusThree = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(3.0, 0.0)));
    public static final Pose2d kCargoFourPlusFour = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(4.0, 0.0)));
    public static final Pose2d kCargoFourPlusFive = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(5.0, 0.0)));

    public static final Pose2d kCargoFourMinusOne = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.0, 0.0)));
    public static final Pose2d kCargoFourMinusTwo = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.0, 0.0)));
    public static final Pose2d kCargoFourMinusThree = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(-3.0, 0.0)));
    public static final Pose2d kCargoFourMinusFour = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(-4.0, 0.0)));
    public static final Pose2d kCargoFourMinusFive = kCargoFourPose.transformBy(Pose2d.fromTranslation(new Translation2d(-5.0, 0.0)));

    // //Rocket Corrections
    public static final Pose2d kRocketOnePlusOne = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.48, -0.875)));
    public static final Pose2d kRocketOnePlusTwo = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.96, -1.75)));
    public static final Pose2d kRocketOnePlusThree = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.44, -2.625)));
    public static final Pose2d kRocketOnePlusFour = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.92, -3.5)));
    public static final Pose2d kRocketOnePlusFive = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.5, -4.375)));

    public static final Pose2d kRocketOneMinusOne = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.48, 0.875)));
    public static final Pose2d kRocketOneMinusTwo = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.96, 1.75)));
    public static final Pose2d kRocketOneMinusThree = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.44, 2.625)));
    public static final Pose2d kRocketOneMinusFour = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.92, 3.5)));
    public static final Pose2d kRocketOneMinusFive = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(2.5, 4.375)));


    public static final Pose2d kRocketThreePlusOne = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.48, -0.875)));
    public static final Pose2d kRocketThreePlusTwo = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.96, -1.75)));
    public static final Pose2d kRocketThreePlusThree = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.44, -2.625)));
    public static final Pose2d kRocketThreePlusFour = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.92, -3.5)));
    public static final Pose2d kRocketThreePlusFive = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(2.5, -4.375)));

    public static final Pose2d kRocketThreeMinusOne = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.48, 0.875)));
    public static final Pose2d kRocketThreeMinusTwo = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.96, 1.75)));
    public static final Pose2d kRocketThreeMinusThree = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.44, 2.625)));
    public static final Pose2d kRocketThreeMinusFour = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.92, 3.5)));
    public static final Pose2d kRocketThreeMinusFive = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.5, 4.375)));

    //Loading Station Corrections
    public static final Pose2d kLoadingStationPlusOne = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -1.0)));
    public static final Pose2d kLoadingStationPlusTwo = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -2.0)));
    public static final Pose2d kLoadingStationPlusThree = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -3.0)));
    public static final Pose2d kLoadingStationPlusFour = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -4.0)));
    public static final Pose2d kLoadingStationPlusFive = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, -5.0)));

    public static final Pose2d kLoadingStationMinusOne = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 1.0)));
    public static final Pose2d kLoadingStationMinusTwo = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 2.0)));
    public static final Pose2d kLoadingStationMinusThree = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 3.0)));
    public static final Pose2d kLoadingStationMinusFour = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 4.0)));
    public static final Pose2d kLoadingStationMinusFive = kLoadingStationPose.transformBy(Pose2d.fromTranslation(new Translation2d(0.0, 5.0)));
    
    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        //Lineup to First Goal Trajectories

        // public final MirroredTrajectory levelOneToOppositeCargoOneLineup;
        // public final MirroredTrajectory levelTwoToOppositeCargoOneLineup;

        public final MirroredTrajectory levelOneToCargoTwoLineup;
        public final MirroredTrajectory levelTwoToCargoTwoLineup;

        public final MirroredTrajectory levelOneToRocketOneLineup;
        public final MirroredTrajectory levelTwoToRocketOneLineup;
        
        public final MirroredTrajectory levelOneToRocketThreeLineup;
        public final MirroredTrajectory levelTwoToRocketThreeLineup;

        // public final MirroredTrajectory oppositeCargoOneLineupToOppositeCargoOne;
        // public final MirroredTrajectory cargoOneLineupToCargoOne;
        public final MirroredTrajectory cargoTwoLineupToCargoTwo;
        public final MirroredTrajectory cargoThreeLineupToCargoThree;
        // public final MirroredTrajectory cargoFourLineupToCargoFour;
        public final MirroredTrajectory rocketOneLineupToRocketOne;
        public final MirroredTrajectory rocketThreeLineupToRocketThree;
        public final MirroredTrajectory loadingStationLineupToLoadingStation;

        // public final MirroredTrajectory oppositeCargoOneCorrectionPlusOne;
        // public final MirroredTrajectory oppositeCargoOneCorrectionPlusTwo;
        // public final MirroredTrajectory oppositeCargoOneCorrectionPlusThree;
        // public final MirroredTrajectory oppositeCargoOneCorrectionPlusFour;
        // public final MirroredTrajectory oppositeCargoOneCorrectionPlusFive;
        
        // public final MirroredTrajectory oppositeCargoOneCorrectionMinusOne;
        // public final MirroredTrajectory oppositeCargoOneCorrectionMinusTwo;
        // public final MirroredTrajectory oppositeCargoOneCorrectionMinusThree;
        // public final MirroredTrajectory oppositeCargoOneCorrectionMinusFour;
        // public final MirroredTrajectory oppositeCargoOneCorrectionMinusFive;
        
        // public final MirroredTrajectory cargoOneCorrectionPlusOne;
        // public final MirroredTrajectory cargoOneCorrectionPlusTwo;
        // public final MirroredTrajectory cargoOneCorrectionPlusThree;
        // public final MirroredTrajectory cargoOneCorrectionPlusFour;
        // public final MirroredTrajectory cargoOneCorrectionPlusFive;
        
        // public final MirroredTrajectory cargoOneCorrectionMinusOne;
        // public final MirroredTrajectory cargoOneCorrectionMinusTwo;
        // public final MirroredTrajectory cargoOneCorrectionMinusThree;
        // public final MirroredTrajectory cargoOneCorrectionMinusFour;
        // public final MirroredTrajectory cargoOneCorrectionMinusFive;

        public final MirroredTrajectory cargoTwoCorrectionPlusOne;
        public final MirroredTrajectory cargoTwoCorrectionPlusTwo;
        public final MirroredTrajectory cargoTwoCorrectionPlusThree;
        public final MirroredTrajectory cargoTwoCorrectionPlusFour;
        public final MirroredTrajectory cargoTwoCorrectionPlusFive;
        
        public final MirroredTrajectory cargoTwoCorrectionMinusOne;
        public final MirroredTrajectory cargoTwoCorrectionMinusTwo;
        public final MirroredTrajectory cargoTwoCorrectionMinusThree;
        public final MirroredTrajectory cargoTwoCorrectionMinusFour;
        public final MirroredTrajectory cargoTwoCorrectionMinusFive;

        public final MirroredTrajectory cargoThreeCorrectionPlusOne;
        public final MirroredTrajectory cargoThreeCorrectionPlusTwo;
        public final MirroredTrajectory cargoThreeCorrectionPlusThree;
        public final MirroredTrajectory cargoThreeCorrectionPlusFour;
        public final MirroredTrajectory cargoThreeCorrectionPlusFive;
        
        public final MirroredTrajectory cargoThreeCorrectionMinusOne;
        public final MirroredTrajectory cargoThreeCorrectionMinusTwo;
        public final MirroredTrajectory cargoThreeCorrectionMinusThree;
        public final MirroredTrajectory cargoThreeCorrectionMinusFour;
        public final MirroredTrajectory cargoThreeCorrectionMinusFive;

        // public final MirroredTrajectory cargoFourCorrectionPlusOne;
        // public final MirroredTrajectory cargoFourCorrectionPlusTwo;
        // public final MirroredTrajectory cargoFourCorrectionPlusThree;
        // public final MirroredTrajectory cargoFourCorrectionPlusFour;
        // public final MirroredTrajectory cargoFourCorrectionPlusFive;
        
        // public final MirroredTrajectory cargoFourCorrectionMinusOne;
        // public final MirroredTrajectory cargoFourCorrectionMinusTwo;
        // public final MirroredTrajectory cargoFourCorrectionMinusThree;
        // public final MirroredTrajectory cargoFourCorrectionMinusFour;
        // public final MirroredTrajectory cargoFourCorrectionMinusFive;

        public final MirroredTrajectory rocketOneCorrectionPlusOne;
        public final MirroredTrajectory rocketOneCorrectionPlusTwo;
        public final MirroredTrajectory rocketOneCorrectionPlusThree;
        public final MirroredTrajectory rocketOneCorrectionPlusFour;
        public final MirroredTrajectory rocketOneCorrectionPlusFive;
        
        public final MirroredTrajectory rocketOneCorrectionMinusOne;
        public final MirroredTrajectory rocketOneCorrectionMinusTwo;
        public final MirroredTrajectory rocketOneCorrectionMinusThree;
        public final MirroredTrajectory rocketOneCorrectionMinusFour;
        public final MirroredTrajectory rocketOneCorrectionMinusFive;

        public final MirroredTrajectory rocketThreeCorrectionPlusOne;
        public final MirroredTrajectory rocketThreeCorrectionPlusTwo;
        public final MirroredTrajectory rocketThreeCorrectionPlusThree;
        public final MirroredTrajectory rocketThreeCorrectionPlusFour;
        public final MirroredTrajectory rocketThreeCorrectionPlusFive;
        
        public final MirroredTrajectory rocketThreeCorrectionMinusOne;
        public final MirroredTrajectory rocketThreeCorrectionMinusTwo;
        public final MirroredTrajectory rocketThreeCorrectionMinusThree;
        public final MirroredTrajectory rocketThreeCorrectionMinusFour;
        public final MirroredTrajectory rocketThreeCorrectionMinusFive;

        public final MirroredTrajectory loadingStationCorrectionPlusOne;
        public final MirroredTrajectory loadingStationCorrectionPlusTwo;
        public final MirroredTrajectory loadingStationCorrectionPlusThree;
        public final MirroredTrajectory loadingStationCorrectionPlusFour;
        public final MirroredTrajectory loadingStationCorrectionPlusFive;
        
        public final MirroredTrajectory loadingStationCorrectionMinusOne;
        public final MirroredTrajectory loadingStationCorrectionMinusTwo;
        public final MirroredTrajectory loadingStationCorrectionMinusThree;
        public final MirroredTrajectory loadingStationCorrectionMinusFour;
        public final MirroredTrajectory loadingStationCorrectionMinusFive;


        public final MirroredTrajectory awayFromRocketOne;
        public final MirroredTrajectory awayFromRocketThree;
        // public final MirroredTrajectory awayFromOppositeCargoOne;
        public final MirroredTrajectory awayFromCargoTwo;
        // public final MirroredTrajectory awayFromCargoThree;

        public final MirroredTrajectory awayFromRocketOneToLoadingStationLineup;
        public final MirroredTrajectory rocketThreeLineupToLoadingStationLineup;

        public final MirroredTrajectory cargoTwoLineupToLoadingStation;
        // public final MirroredTrajectory oppositeCargoOneLineupToLoadingStationLineup;
        // public final MirroredTrajectory cargoThreeLineupToLoadingStationLineup;

        public final MirroredTrajectory loadingStationToCargoThreeLineup;
        public final MirroredTrajectory loadingStationToRocketThreeLineup;

        // public final MirroredTrajectory loadingStationToCargoOneLineup;
        // public final MirroredTrajectory loadingStationToCargoTwoLineup;

        private TrajectorySet() {

            // levelOneToOppositeCargoOneLineup = new MirroredTrajectory(getLevelOneToOppositeCargoOneLineup());
            // levelTwoToOppositeCargoOneLineup = new MirroredTrajectory(getLevelTwoToOppositeCargoOneLineup());

            levelOneToCargoTwoLineup = new MirroredTrajectory(getLevelOneToCargoTwoLineup());
            levelTwoToCargoTwoLineup = new MirroredTrajectory(getLevelTwoToCargoTwoLineup());

            levelOneToRocketOneLineup = new MirroredTrajectory(getLevelOneToRocketOneLineup());
            levelTwoToRocketOneLineup = new MirroredTrajectory(getLevelTwoToRocketOneLineup());

            levelOneToRocketThreeLineup = new MirroredTrajectory(getLevelOneToRocketThreeLineup());
            levelTwoToRocketThreeLineup = new MirroredTrajectory(getLevelTwoToRocketThreeLineup());

            // oppositeCargoOneLineupToOppositeCargoOne = new MirroredTrajectory(getOppositeCargoOneLineupToOppositeCargoOne());
            // cargoOneLineupToCargoOne = new MirroredTrajectory(getCargoOneLineupToCargoOne());
            cargoTwoLineupToCargoTwo = new MirroredTrajectory(getCargoTwoLineupToCargoTwo());
            cargoThreeLineupToCargoThree = new MirroredTrajectory(getCargoThreeLineupToCargoThree());
            // cargoFourLineupToCargoFour = new MirroredTrajectory(getCargoFourLineupToCargoFour());
            rocketOneLineupToRocketOne = new MirroredTrajectory(getRocketOneLineupToRocketOne());
            rocketThreeLineupToRocketThree = new MirroredTrajectory(getRocketThreeLineupToRocketThree());
            loadingStationLineupToLoadingStation = new MirroredTrajectory(getLoadingStationLineupToLoadingStation());

            // oppositeCargoOneCorrectionPlusOne = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusOne());
            // oppositeCargoOneCorrectionPlusTwo = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusTwo());
            // oppositeCargoOneCorrectionPlusThree = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusThree());
            // oppositeCargoOneCorrectionPlusFour = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusFour());
            // oppositeCargoOneCorrectionPlusFive = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusFive());

            // oppositeCargoOneCorrectionMinusOne = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusOne());
            // oppositeCargoOneCorrectionMinusTwo = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusTwo());
            // oppositeCargoOneCorrectionMinusThree = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusThree());
            // oppositeCargoOneCorrectionMinusFour = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusFour());
            // oppositeCargoOneCorrectionMinusFive = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusFive());

            // cargoOneCorrectionPlusOne = new MirroredTrajectory(getCargoOneCorrectionPlusOne());
            // cargoOneCorrectionPlusTwo = new MirroredTrajectory(getCargoOneCorrectionPlusTwo());
            // cargoOneCorrectionPlusThree = new MirroredTrajectory(getCargoOneCorrectionPlusThree());
            // cargoOneCorrectionPlusFour = new MirroredTrajectory(getCargoOneCorrectionPlusFour());
            // cargoOneCorrectionPlusFive = new MirroredTrajectory(getCargoOneCorrectionPlusFive());

            // cargoOneCorrectionMinusOne = new MirroredTrajectory(getCargoOneCorrectionMinusOne());
            // cargoOneCorrectionMinusTwo = new MirroredTrajectory(getCargoOneCorrectionMinusTwo());
            // cargoOneCorrectionMinusThree = new MirroredTrajectory(getCargoOneCorrectionMinusThree());
            // cargoOneCorrectionMinusFour = new MirroredTrajectory(getCargoOneCorrectionMinusFour());
            // cargoOneCorrectionMinusFive = new MirroredTrajectory(getCargoOneCorrectionMinusFive());

            cargoTwoCorrectionPlusOne = new MirroredTrajectory(getCargoTwoCorrectionPlusOne());
            cargoTwoCorrectionPlusTwo = new MirroredTrajectory(getCargoTwoCorrectionPlusTwo());
            cargoTwoCorrectionPlusThree = new MirroredTrajectory(getCargoTwoCorrectionPlusThree());
            cargoTwoCorrectionPlusFour = new MirroredTrajectory(getCargoTwoCorrectionPlusFour());
            cargoTwoCorrectionPlusFive = new MirroredTrajectory(getCargoTwoCorrectionPlusFive());

            cargoTwoCorrectionMinusOne = new MirroredTrajectory(getCargoTwoCorrectionMinusOne());
            cargoTwoCorrectionMinusTwo = new MirroredTrajectory(getCargoTwoCorrectionMinusTwo());
            cargoTwoCorrectionMinusThree = new MirroredTrajectory(getCargoTwoCorrectionMinusThree());
            cargoTwoCorrectionMinusFour = new MirroredTrajectory(getCargoTwoCorrectionMinusFour());
            cargoTwoCorrectionMinusFive = new MirroredTrajectory(getCargoTwoCorrectionMinusFive());
            
            cargoThreeCorrectionPlusOne = new MirroredTrajectory(getCargoThreeCorrectionPlusOne());
            cargoThreeCorrectionPlusTwo = new MirroredTrajectory(getCargoThreeCorrectionPlusTwo());
            cargoThreeCorrectionPlusThree = new MirroredTrajectory(getCargoThreeCorrectionPlusThree());
            cargoThreeCorrectionPlusFour = new MirroredTrajectory(getCargoThreeCorrectionPlusFour());
            cargoThreeCorrectionPlusFive = new MirroredTrajectory(getCargoThreeCorrectionPlusFive());

            cargoThreeCorrectionMinusOne = new MirroredTrajectory(getCargoThreeCorrectionMinusOne());
            cargoThreeCorrectionMinusTwo = new MirroredTrajectory(getCargoThreeCorrectionMinusTwo());
            cargoThreeCorrectionMinusThree = new MirroredTrajectory(getCargoThreeCorrectionMinusThree());
            cargoThreeCorrectionMinusFour = new MirroredTrajectory(getCargoThreeCorrectionMinusFour());
            cargoThreeCorrectionMinusFive = new MirroredTrajectory(getCargoThreeCorrectionMinusFive());

            // cargoFourCorrectionPlusOne = new MirroredTrajectory(getCargoFourCorrectionPlusOne());
            // cargoFourCorrectionPlusTwo = new MirroredTrajectory(getCargoFourCorrectionPlusTwo());
            // cargoFourCorrectionPlusThree = new MirroredTrajectory(getCargoFourCorrectionPlusThree());
            // cargoFourCorrectionPlusFour = new MirroredTrajectory(getCargoFourCorrectionPlusFour());
            // cargoFourCorrectionPlusFive = new MirroredTrajectory(getCargoFourCorrectionPlusFive());

            // cargoFourCorrectionMinusOne = new MirroredTrajectory(getCargoFourCorrectionMinusOne());
            // cargoFourCorrectionMinusTwo = new MirroredTrajectory(getCargoFourCorrectionMinusTwo());
            // cargoFourCorrectionMinusThree = new MirroredTrajectory(getCargoFourCorrectionMinusThree());
            // cargoFourCorrectionMinusFour = new MirroredTrajectory(getCargoFourCorrectionMinusFour());
            // cargoFourCorrectionMinusFive = new MirroredTrajectory(getCargoFourCorrectionMinusFive());

            rocketOneCorrectionPlusOne = new MirroredTrajectory(getRocketOneCorrectionPlusOne());
            rocketOneCorrectionPlusTwo = new MirroredTrajectory(getRocketOneCorrectionPlusTwo());
            rocketOneCorrectionPlusThree = new MirroredTrajectory(getRocketOneCorrectionPlusThree());
            rocketOneCorrectionPlusFour = new MirroredTrajectory(getRocketOneCorrectionPlusFour());
            rocketOneCorrectionPlusFive = new MirroredTrajectory(getRocketOneCorrectionPlusFive());

            rocketOneCorrectionMinusOne = new MirroredTrajectory(getRocketOneCorrectionMinusOne());
            rocketOneCorrectionMinusTwo = new MirroredTrajectory(getRocketOneCorrectionMinusTwo());
            rocketOneCorrectionMinusThree = new MirroredTrajectory(getRocketOneCorrectionMinusThree());
            rocketOneCorrectionMinusFour = new MirroredTrajectory(getRocketOneCorrectionMinusFour());
            rocketOneCorrectionMinusFive = new MirroredTrajectory(getRocketOneCorrectionMinusFive());

            rocketThreeCorrectionPlusOne = new MirroredTrajectory(getRocketThreeCorrectionPlusOne());
            rocketThreeCorrectionPlusTwo = new MirroredTrajectory(getRocketThreeCorrectionPlusTwo());
            rocketThreeCorrectionPlusThree = new MirroredTrajectory(getRocketThreeCorrectionPlusThree());
            rocketThreeCorrectionPlusFour = new MirroredTrajectory(getRocketThreeCorrectionPlusFour());
            rocketThreeCorrectionPlusFive = new MirroredTrajectory(getRocketThreeCorrectionPlusFive());

            rocketThreeCorrectionMinusOne = new MirroredTrajectory(getRocketThreeCorrectionMinusOne());
            rocketThreeCorrectionMinusTwo = new MirroredTrajectory(getRocketThreeCorrectionMinusTwo());
            rocketThreeCorrectionMinusThree = new MirroredTrajectory(getRocketThreeCorrectionMinusThree());
            rocketThreeCorrectionMinusFour = new MirroredTrajectory(getRocketThreeCorrectionMinusFour());
            rocketThreeCorrectionMinusFive = new MirroredTrajectory(getRocketThreeCorrectionMinusFive());

            loadingStationCorrectionPlusOne = new MirroredTrajectory(getLoadingStationCorrectionPlusOne());
            loadingStationCorrectionPlusTwo = new MirroredTrajectory(getLoadingStationCorrectionPlusTwo());
            loadingStationCorrectionPlusThree = new MirroredTrajectory(getLoadingStationCorrectionPlusThree());
            loadingStationCorrectionPlusFour = new MirroredTrajectory(getLoadingStationCorrectionPlusFour());
            loadingStationCorrectionPlusFive = new MirroredTrajectory(getLoadingStationCorrectionPlusFive());

            loadingStationCorrectionMinusOne = new MirroredTrajectory(getLoadingStationCorrectionMinusOne());
            loadingStationCorrectionMinusTwo = new MirroredTrajectory(getLoadingStationCorrectionMinusTwo());
            loadingStationCorrectionMinusThree = new MirroredTrajectory(getLoadingStationCorrectionMinusThree());
            loadingStationCorrectionMinusFour = new MirroredTrajectory(getLoadingStationCorrectionMinusFour());
            loadingStationCorrectionMinusFive = new MirroredTrajectory(getLoadingStationCorrectionMinusFive());

            // awayFromOppositeCargoOne = new MirroredTrajectory(getAwayFromOppositeCargoOne());
            awayFromCargoTwo = new MirroredTrajectory(getAwayFromCargoTwo());
            // awayFromCargoThree = new MirroredTrajectory(getAwayFromCargoThree());
            awayFromRocketOne = new MirroredTrajectory(getAwayFromRocketOne());
            awayFromRocketThree = new MirroredTrajectory(getAwayFromRocketThree());

            // oppositeCargoOneLineupToLoadingStationLineup = new MirroredTrajectory(getOppositeCargoOneLineupToLoadingStationLineup());
            // cargoThreeLineupToLoadingStationLineup = new MirroredTrajectory(getCargoThreeLineupToLoadingStationLineup());
            cargoTwoLineupToLoadingStation = new MirroredTrajectory(getCargoTwoLineupToLoadingStation());
            awayFromRocketOneToLoadingStationLineup = new MirroredTrajectory(getAwayFromRocketOneToLoadingStationLineup());
            rocketThreeLineupToLoadingStationLineup = new MirroredTrajectory(getRocketThreeLineupToLoadingStationLineup());

            // loadingStationToCargoOneLineup = new MirroredTrajectory(getLoadingStationToCargoOneLineup());
            // loadingStationToCargoTwoLineup = new MirroredTrajectory(getLoadingStationToCargoTwoLineup());
            loadingStationToCargoThreeLineup = new MirroredTrajectory(getLoadingStationToCargoThreeLineup());
            loadingStationToRocketThreeLineup = new MirroredTrajectory(getLoadingStationToRocketThreeLineup());
        }

        //Lineup to first goals
        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToOppositeCargoOneLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevelOneStartPose);
        //     waypoints.add(kOffHabPlatform);
        //     waypoints.add(kOppositeCargoOneLineupPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToOppositeCargoOneLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevelTwoStartPose);
        //     waypoints.add(kOffHabPlatform);
        //     waypoints.add(kOppositeCargoOneLineupPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoTwoLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartBackwardPose);
            waypoints.add(kOffHabPlatformBackwards);
            waypoints.add(new Pose2d(new Translation2d(225.0, -55.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoTwoLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToCargoTwoLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartBackwardPose);
            waypoints.add(kOffHabPlatformBackwards);
            waypoints.add(new Pose2d(new Translation2d(225.0, -50.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoTwoLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToRocketOneLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            // waypoints.add(new Pose2d(new Translation2d(145.0, -104.0), Rotation2d.fromDegrees(330.0)));
            waypoints.add(kRocketOneLineupPose);
            // waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToRocketOneLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToRocketThreeLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartBackwardPose);
            waypoints.add(kOffHabPlatformBackwards);
            waypoints.add(new Pose2d(new Translation2d(250.0, -115.0), Rotation2d.fromDegrees(150.0)));
            waypoints.add(kRocketThreeLineupPose);
            // waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToRocketThreeLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //no corrections needed trajectories
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneLineupToOppositeCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneLineupToCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoLineupToCargoTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeLineupToCargoThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourLineupToCargoFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneLineupToRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeLineupToRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationLineupToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // //correction to opposite cargo one with limelight offset
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionPlusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePlusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionPlusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePlusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionPlusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePlusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionPlusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePlusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionPlusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOnePlusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionMinusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOneMinusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionMinusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOneMinusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionMinusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOneMinusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionMinusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOneMinusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneCorrectionMinusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kOppositeCargoOneMinusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // //correction to cargo one with limelight offset
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionPlusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePlusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionPlusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePlusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionPlusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePlusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionPlusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePlusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionPlusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOnePlusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionMinusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOneMinusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionMinusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOneMinusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionMinusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOneMinusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionMinusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOneMinusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoOneCorrectionMinusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoOneLineupPose);
        //     waypoints.add(kCargoOneMinusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        //correction to cargo two from limelight offset
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionPlusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPlusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionPlusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPlusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionPlusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPlusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionPlusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPlusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionPlusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoPlusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionMinusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoMinusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionMinusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoMinusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionMinusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoMinusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionMinusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoMinusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoCorrectionMinusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoMinusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // //correction to cargo three from limelight offset
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionPlusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePlusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionPlusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePlusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionPlusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePlusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionPlusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePlusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionPlusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreePlusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionMinusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreeMinusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionMinusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreeMinusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionMinusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreeMinusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionMinusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreeMinusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeCorrectionMinusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoThreeLineupPose);
            waypoints.add(kCargoThreeMinusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // //correction to cargo four from limelight offset
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionPlusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPlusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionPlusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPlusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionPlusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPlusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionPlusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPlusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionPlusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourPlusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionMinusOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourMinusOne);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionMinusTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourMinusTwo);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionMinusThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourMinusThree);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionMinusFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourMinusFour);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoFourCorrectionMinusFive() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoFourLineupPose);
        //     waypoints.add(kCargoFourMinusFive);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

         //correction to rocket three from limelight offset
         private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionPlusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePlusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionPlusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePlusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionPlusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePlusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionPlusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePlusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionPlusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePlusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionMinusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOneMinusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,  kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionMinusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOneMinusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionMinusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOneMinusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,  kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionMinusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOneMinusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneCorrectionMinusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOneMinusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //correction to rocket three from limelight offset
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionPlusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePlusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionPlusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePlusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionPlusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePlusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionPlusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePlusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionPlusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreePlusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionMinusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreeMinusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,  kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionMinusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreeMinusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionMinusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreeMinusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0,  kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionMinusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreeMinusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeCorrectionMinusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreeLineupPose);
            waypoints.add(kRocketThreeMinusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
            0.0, 40.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //correction to loading station from limelight offset
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionPlusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPlusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionPlusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPlusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionPlusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPlusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionPlusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPlusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionPlusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationPlusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionMinusOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationMinusOne);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionMinusTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationMinusTwo);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionMinusThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationMinusThree);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionMinusFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationMinusFour);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationCorrectionMinusFive() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(kLoadingStationMinusFive);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //retreat to lineup waypoint
        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromOppositeCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOnePose);
        //     waypoints.add(kOppositeCargoOneLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromCargoTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kCargoTwoPose);
            waypoints.add(kCargoTwoBackupPose);
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoLineupToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kCargoTwoBackupPose);
            waypoints.add(new Pose2d(new Translation2d(200.0, -55.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kComingToLoadingStation);
            waypoints.add(kLoadingStationLineupPose);
            // waypoints.add(kLoadingStationPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromCargoThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoThreePose);
        //     waypoints.add(kCargoThreeLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketOnePose);
            waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRocketThreePose);
            waypoints.add(new Pose2d(new Translation2d(289.0, -119.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //lineup waypoint to loading station
        // private Trajectory<TimedState<Pose2dWithCurvature>> getOppositeCargoOneLineupToLoadingStationLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOneLineupPose);
        //     waypoints.add(kLoadingStationLineupPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeLineupToLoadingStationLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoThreeLineupPose);
        //     waypoints.add(kLoadingStationLineupPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOneToLoadingStationLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));
            waypoints.add(kLoadingStationLineupPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeLineupToLoadingStationLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(289.0, -119.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(200.0, -115.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kLoadingStationLineupPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        //loading station to lineup waypoint
        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoOneLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLoadingStationPose);
        //     waypoints.add(kCargoOneLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoTwoLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLoadingStationPose);
        //     waypoints.add(kCargoTwoLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoThreeLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(kLoadingStationLineupPose);
            waypoints.add(new Pose2d(new Translation2d(265.0, -55.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60.0 )),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketThreeLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(new Pose2d(new Translation2d(230.0, -110.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getStraightTest() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(166.0, -50.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kLevel1RightCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2RightCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel  , kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLeftLevel1StartPose);
        //     waypoints.add(kLevel1LeftCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel2toCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLeftLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2LeftCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kLevel1RightRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(66.0, 50.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(220.0, 100.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(295.0, 120.0), Rotation2d.fromDegrees(150.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2RightRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel  , kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2RightRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel  , kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLeftLevel1StartPose);
        //     waypoints.add(kLevel1LeftRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel2toRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLeftLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2LeftRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(kRightRocketThreePlusOne);
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -122.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -123.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus4() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -124.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus5() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -125.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus1() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -119.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus2() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -118.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus3() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -117.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus4() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -116.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus5() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -115.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(265.0, -120.0), Rotation2d.fromDegrees(210.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRightRocketOneToTurn() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevel2RightRocketOnePose);
        //     waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLeftRocketOneToTurn() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevel2LeftRocketOnePose);
        //     waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getEndRightTurnToLoadStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));
        //     waypoints.add(kRightLoadingStationPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getEndLeftTurnToLoadStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));
        //     waypoints.add(kLeftLoadingStationPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kRightCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(kRightCargoOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kRightCargoTwoPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(kRightCargoTwoPose);
            
        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kRightCargoThreePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(kRightOffLevel2Pose);
        //     waypoints.add(new Pose2d(new Translation2d(284.0, -45.0), Rotation2d.fromDegrees(90.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kRightCargoFourPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(kRightCargoFourPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel1RightRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kLevel2RightRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(kRightRocketTwoPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(kRightRocketTwoPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel1StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(225.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(kRightRocketThreePose);
        //     waypoints.add(new Pose2d(new Translation2d(250.0, -135.0), Rotation2d.fromDegrees(210.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightLevel2StartPose);
        //     waypoints.add(new Pose2d(new Translation2d(225.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(kRightRocketThreePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // // //Score second game piece trajectories

        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeTo90Turn() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRightCargoThreePose);
        //     waypoints.add(new Pose2d(new Translation2d(301.0, -65.0), Rotation2d.fromDegrees(150.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeToTurn() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(250.0, -135.0), Rotation2d.fromDegrees(210.0)));
        //     waypoints.add(new Pose2d(new Translation2d(280.0, -120.0), Rotation2d.fromDegrees(180.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getEndTurnToLoadingStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(311.0, -75.0), Rotation2d.fromDegrees(150.0)));
        //     waypoints.add(new Pose2d(new Translation2d(275.0, -75.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(25.0, -125.0), Rotation2d.fromDegrees(180.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getEndRocketTurnToLoadingStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(280.0, -120.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(175.0, -120.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(25.0, -148.0), Rotation2d.fromDegrees(180.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoFour() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     // waypoints.add(kRightLoadingStationPose);
        //     // waypoints.add(new Pose2d(new Translation2d(175.0, -100.0), Rotation2d.fromDegrees(0.0)));
        //     // waypoints.add(kRightCargoTwoPose);
        //     waypoints.add(new Pose2d(new Translation2d(25.0, -125.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(250.0, -65.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(315.0, -95.0), Rotation2d.fromDegrees(80.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(25.0, -148.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(175.0, -123.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(235.0, -98.0), Rotation2d.fromDegrees(275.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
    }
}
