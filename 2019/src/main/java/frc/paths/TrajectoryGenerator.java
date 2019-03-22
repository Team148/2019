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
    private static final double kMaxVelocity = 100.0;
    private static final double kMaxAccel = 50.0;
    // private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    // private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxCentripetalAccelElevatorDown = 30.0;
    private static final double kMaxCentripetalAccel = 80.0;
    private static final double kMaxVoltage = 9.0;
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
    public static final Pose2d kLevelTwoStartPose = new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevelTwoStartBackwardPose = new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(180.0));
 
    // //Off Level 2
    public static final Pose2d kOffHabPlatform = new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(180.0));

    // //Scoring Poses
    // //Cargo One
    public static final Pose2d kCargoOnePose = new Pose2d(new Translation2d(205.0, -11.0), Rotation2d.fromDegrees(0.0));

    // // Opposite Side Cargo One
    public static final Pose2d kOppositeCargoOnePose = new Pose2d(new Translation2d(205.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two
    public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(260.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Three
    public static final Pose2d kCargoThreePose = new Pose2d(new Translation2d(281.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Four
    public static final Pose2d kCargoFourPose = new Pose2d(new Translation2d(303.0, -40.0), Rotation2d.fromDegrees(90.0));

    // //Rocket One
    public static final Pose2d kRocketOnePose = new Pose2d(new Translation2d(198.0, -135.0), Rotation2d.fromDegrees(330.0));

    // //Rocket Three
    public static final Pose2d kRocketThreePose = new Pose2d(new Translation2d(260.0, -135.0), Rotation2d.fromDegrees(210.0));

    // //Loading Station Poses
    public static final Pose2d kLoadingStationPose = new Pose2d(new Translation2d(18.0, -135.0), Rotation2d.fromDegrees(180.0)); 

    // //Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kCargoOneLineupPose = new Pose2d(new Translation2d(175.0, -11.0), Rotation2d.fromDegrees(0.0));

    // //Opposite Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kOppositeCargoOneLineupPose = new Pose2d(new Translation2d(175.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two Lineup (for Vision Tracking)
    public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(260.0, -70.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Three Lineup (for Vision Tracking)
    public static final Pose2d kCargoThreeLineupPose = new Pose2d(new Translation2d(281.0, -70.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Four Lineup (for Vision Tracking)
    public static final Pose2d kCargoFourLineupPose = new Pose2d(new Translation2d(303.0, -70.0), Rotation2d.fromDegrees(90.0));

    // //Rocket One Lineup (for Vision Tracking)
    public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(171.0, -119.0), Rotation2d.fromDegrees(330.0));

    // //Rocket Three Lineup (for Vision Tracking)
    public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -119.0), Rotation2d.fromDegrees(210.0));

    // //Loading Station Lineup (for Vision Tracking)
    public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(48.0, -135.0), Rotation2d.fromDegrees(180.0));

    // //Correction Poses (Minus 1 is 1 inch left, Plus 1 is 1 inch right)
    // //Rocket Corrections
    public static final Pose2d kRocketOnePlusOne = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.48, 0.875)));
    public static final Pose2d kRocketOnePlusTwo = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.96, 1.75)));
    public static final Pose2d kRocketOnePlusThree = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.44, 2.625)));
    public static final Pose2d kRocketOnePlusFour = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.92, 3.5)));
    public static final Pose2d kRocketOnePlusFive = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(2.5, 4.375)));

    public static final Pose2d kRocketOneMinusOne = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.48, 0.875)));
    public static final Pose2d kRocketoneMinusTwo = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.96, 1.75)));
    public static final Pose2d kRocketOneMinusThree = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.44, 2.625)));
    public static final Pose2d kRocketOneMinusFour = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.92, 3.5)));
    public static final Pose2d kRocketOneMinusFive = kRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.5, 4.375)));


    public static final Pose2d kRocketThreePlusOne = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.48, 0.875)));
    public static final Pose2d kRocketThreePlusTwo = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.96, 1.75)));
    public static final Pose2d kRocketThreePlusThree = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.44, 2.625)));
    public static final Pose2d kRocketThreePlusFour = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(1.92, 3.5)));
    public static final Pose2d kRocketThreePlusFive = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(2.5, 4.375)));

    public static final Pose2d kRocketThreeMinusOne = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.48, 0.875)));
    public static final Pose2d kRocketThreeMinusTwo = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.96, 1.75)));
    public static final Pose2d kRocketThreeMinusThree = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.44, 2.625)));
    public static final Pose2d kRocketThreeMinusFour = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-1.92, 3.5)));
    public static final Pose2d kRocketThreeMinusFive = kRocketThreePose.transformBy(Pose2d.fromTranslation(new Translation2d(-2.5, 4.375)));

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

        public final MirroredTrajectory levelOneToOppositeCargoOneLineup;
        public final MirroredTrajectory levelTwoToOppositeCargoOneLineup;

        public final MirroredTrajectory levelOneToCargoThreeLineup;
        public final MirroredTrajectory levelTwoToCargoThreeLineup;
        
        public final MirroredTrajectory levelOneToRocketThreeLineup;
        public final MirroredTrajectory levelTwoToRocketThreeLineup;

        public final MirroredTrajectory oppositeCargoOneCorrectionPlusOne;
        public final MirroredTrajectory oppositeCargoOneCorrectionPlusTwo;
        public final MirroredTrajectory oppositeCargoOneCorrectionPlusThree;
        public final MirroredTrajectory oppositeCargoOneCorrectionPlusFour;
        public final MirroredTrajectory oppositeCargoOneCorrectionPlusFive;
        
        public final MirroredTrajectory oppositeCargoOneCorrectionMinusOne;
        public final MirroredTrajectory oppositeCargoOneCorrectionMinusTwo;
        public final MirroredTrajectory oppositeCargoOneCorrectionMinusThree;
        public final MirroredTrajectory oppositeCargoOneCorrectionMinusFour;
        public final MirroredTrajectory oppositeCargoOneCorrectionMinusFive;
        
        public final MirroredTrajectory cargoOneCorrectionPlusOne;
        public final MirroredTrajectory cargoOneCorrectionPlusTwo;
        public final MirroredTrajectory cargoOneCorrectionPlusThree;
        public final MirroredTrajectory cargoOneCorrectionPlusFour;
        public final MirroredTrajectory cargoOneCorrectionPlusFive;
        
        public final MirroredTrajectory cargoOneCorrectionMinusOne;
        public final MirroredTrajectory cargoOneCorrectionMinusTwo;
        public final MirroredTrajectory cargoOneCorrectionMinusThree;
        public final MirroredTrajectory cargoOneCorrectionMinusFour;
        public final MirroredTrajectory cargoOneCorrectionMinusFive;

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

        public final MirroredTrajectory cargoFourCorrectionPlusOne;
        public final MirroredTrajectory cargoFourCorrectionPlusTwo;
        public final MirroredTrajectory cargoFourCorrectionPlusThree;
        public final MirroredTrajectory cargoFourCorrectionPlusFour;
        public final MirroredTrajectory cargoFourCorrectionPlusFive;
        
        public final MirroredTrajectory cargoFourCorrectionMinusOne;
        public final MirroredTrajectory cargoFourCorrectionMinusTwo;
        public final MirroredTrajectory cargoFourCorrectionMinusThree;
        public final MirroredTrajectory cargoFourCorrectionMinusFour;
        public final MirroredTrajectory cargoFourCorrectionMinusFive;

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

        public final MirroredTrajectory rocketThreeTurnAway;
        public final MirroredTrajectory oppositeCargoOneTurnAway;
        public final MirroredTrajectory cargoThreeTurnAway;

        public final MirroredTrajectory endRocketTurnToLoadingStationLineup;
        public final MirroredTrajectory endOppositeCargoTurnToLoadingStationLineup;
        public final MirroredTrajectory endCargoThreeTurnToLoadingStationLineup;

        public final MirroredTrajectory loadingStationToRocketThreeLineup;
        public final MirroredTrajectory loadingStationToCargoOneLineup;
        public final MirroredTrajectory loadingStationToCargoTwoLineup;

        private TrajectorySet() {

            levelOneToOppositeCargoOneLineup = new MirroredTrajectory(getLevelOneToOppositeCargoOneLineup());
            levelTwoToOppositeCargoOneLineup = new MirroredTrajectory(getLevelTwoToOppositeCargoOneLineup());

            levelOneToCargoThreeLineup = new MirroredTrajectory(getLevelOneToCargoThreeLineup());
            levelTwoToCargoThreeLineup = new MirroredTrajectory(getLevelTwoToCargoThreeLineup());

            levelOneToRocketThreeLineup = new MirroredTrajectory(getLevelOneToRocketThreeLineup());
            levelTwoToRocketThreeLineup = new MirroredTrajectory(getLevelTwoToRocketThreeLineup());

            oppositeCargoOneCorrectionPlusOne = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusOne());
            oppositeCargoOneCorrectionPlusTwo = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusTwo());
            oppositeCargoOneCorrectionPlusThree = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusThree());
            oppositeCargoOneCorrectionPlusFour = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusFour());
            oppositeCargoOneCorrectionPlusFive = new MirroredTrajectory(getOppositeCargoOneCorrectionPlusFive());

            oppositeCargoOneCorrectionMinusOne = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusOne());
            oppositeCargoOneCorrectionMinusTwo = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusTwo());
            oppositeCargoOneCorrectionMinusThree = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusThree());
            oppositeCargoOneCorrectionMinusFour = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusFour());
            oppositeCargoOneCorrectionMinusFive = new MirroredTrajectory(getOppositeCargoOneCorrectionMinusFive());

            cargoOneCorrectionPlusOne = new MirroredTrajectory(getCargoOneCorrectionPlusOne());
            cargoOneCorrectionPlusTwo = new MirroredTrajectory(getCargoOneCorrectionPlusTwo());
            cargoOneCorrectionPlusThree = new MirroredTrajectory(getCargoOneCorrectionPlusThree());
            cargoOneCorrectionPlusFour = new MirroredTrajectory(getargoOneCorrectionPlusFour());
            cargoOneCorrectionPlusFive = new MirroredTrajectory(getCargoOneCorrectionPlusFive());

            cargoOneCorrectionMinusOne = new MirroredTrajectory(getCargoOneCorrectionMinusOne());
            cargoOneCorrectionMinusTwo = new MirroredTrajectory(getCargoOneCorrectionMinusTwo());
            cargoOneCorrectionMinusThree = new MirroredTrajectory(getCargoOneCorrectionMinusThree());
            cargoOneCorrectionMinusFour = new MirroredTrajectory(getCargoOneCorrectionMinusFour());
            cargoOneCorrectionMinusFive = new MirroredTrajectory(getCargoOneCorrectionMinusFive());

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
            cargoThreeCorrectionPlusFour = new MirroredTrajectory(getargoThreeCorrectionPlusFour());
            cargoThreeCorrectionPlusFive = new MirroredTrajectory(getCargoThreeCorrectionPlusFive());

            cargoThreeCorrectionMinusOne = new MirroredTrajectory(getCargoThreeCorrectionMinusOne());
            cargoThreeCorrectionMinusTwo = new MirroredTrajectory(getCargoThreeCorrectionMinusTwo());
            cargoThreeCorrectionMinusThree = new MirroredTrajectory(getCargoThreeCorrectionMinusThree());
            cargoThreeCorrectionMinusFour = new MirroredTrajectory(getCargoThreeCorrectionMinusFour());
            cargoThreeCorrectionMinusFive = new MirroredTrajectory(getCargoThreeCorrectionMinusFive());

            cargoFourCorrectionPlusOne = new MirroredTrajectory(getCargoFourCorrectionPlusOne());
            cargoFourCorrectionPlusTwo = new MirroredTrajectory(getCargoFourCorrectionPlusTwo());
            cargoFourCorrectionPlusThree = new MirroredTrajectory(getCargoFourCorrectionPlusThree());
            cargoFourCorrectionPlusFour = new MirroredTrajectory(getCargoFourCorrectionPlusFour());
            cargoFourCorrectionPlusFive = new MirroredTrajectory(getCargoFourCorrectionPlusFive());

            cargoFourCorrectionMinusOne = new MirroredTrajectory(getCargoFourCorrectionMinusOne());
            cargoFourCorrectionMinusTwo = new MirroredTrajectory(getCargoFourCorrectionMinusTwo());
            cargoFourCorrectionMinusThree = new MirroredTrajectory(getCargoFourCorrectionMinusThree());
            cargoFourCorrectionMinusFour = new MirroredTrajectory(getCargoFourCorrectionMinusFour());
            cargoFourCorrectionMinusFive = new MirroredTrajectory(getCargoFourCorrectionMinusFive());

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

            rocketThreeTurnAway = new MirroredTrajectory(getRocketThreeTurnAway());
            oppositeCargoOneTurnAway = new MirroredTrajectory(getOppositeCargoOneTurnAway());
            cargoThreeTurnAway = new MirroredTrajectory(getCargoThreeTurnAway());

            endRocketTurnToLoadingStationLineup = new MirroredTrajectory(getEndRocketTurnToLoadingStationLineup());
            endOppositeCargoTurnToLoadingStationLineup = new MirroredTrajectory(getEndOppositeCargoTurnToLoadingStationLineup());
            endCargoThreeTurnToLoadingStationLineup = new MirroredTrajectory(getEndCargoThreeTurnToLoadingStationLineup());

            loadingStationToRocketThreeLineup = new MirroredTrajectory(getLoadingStationToRocketThreeLineup());
            loadingStationToCargoOneLineup = new MirroredTrajectory(getLoadingStationToCargoOneLineup());
            loadingStationToCargoTwoLineup = new MirroredTrajectory(getLoadingStationToCargoTwoLineup());
        }

        //Score first game piece trajectories

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightTest() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(new Pose2d(new Translation2d(166.0, -50.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kLevel1RightCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2RightCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel  , kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLevel1StartPose);
            waypoints.add(kLevel1LeftCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel2toCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2LeftCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kLevel1RightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(66.0, 50.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(220.0, 100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(295.0, 120.0), Rotation2d.fromDegrees(150.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2RightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel  , kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel2toRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2RightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel  , kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel1toRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLevel1StartPose);
            waypoints.add(kLevel1LeftRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLevel2toRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2LeftRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(kRightRocketThreePlusOne);
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -122.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -123.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -124.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionPlus5() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -125.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -119.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -118.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -117.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -116.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeWithCorrectionMinus5() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -115.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(295.0, -120.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(265.0, -120.0), Rotation2d.fromDegrees(210.0)));
            // waypoints.add(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(new Pose2d(new Translation2d(30.0, -3.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightRocketOneToTurn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2RightRocketOnePose);
            waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftRocketOneToTurn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel2LeftRocketOnePose);
            waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndRightTurnToLoadStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));
            waypoints.add(kRightLoadingStationPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndLeftTurnToLoadStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));
            waypoints.add(kLeftLoadingStationPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kRightCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightCargoOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kRightCargoTwoPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightCargoTwoPose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kRightCargoThreePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightOffLevel2Pose);
            waypoints.add(new Pose2d(new Translation2d(284.0, -45.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToCargoFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kRightCargoFourPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToCargoFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightCargoFourPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel1RightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kLevel2RightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kRightRocketTwoPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightRocketTwoPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel1ToRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(new Pose2d(new Translation2d(225.0, -50.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(kRightRocketThreePose);
            waypoints.add(new Pose2d(new Translation2d(250.0, -135.0), Rotation2d.fromDegrees(210.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketThree() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(new Pose2d(new Translation2d(225.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(kRightRocketThreePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // //Score second game piece trajectories

        private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeTo90Turn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightCargoThreePose);
            waypoints.add(new Pose2d(new Translation2d(301.0, -65.0), Rotation2d.fromDegrees(150.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeToTurn() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(250.0, -135.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(280.0, -120.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndTurnToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(311.0, -75.0), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(275.0, -75.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(25.0, -125.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndRocketTurnToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(280.0, -120.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(175.0, -120.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(25.0, -148.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoFour() {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(kRightLoadingStationPose);
            // waypoints.add(new Pose2d(new Translation2d(175.0, -100.0), Rotation2d.fromDegrees(0.0)));
            // waypoints.add(kRightCargoTwoPose);
            waypoints.add(new Pose2d(new Translation2d(25.0, -125.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(250.0, -65.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(315.0, -95.0), Rotation2d.fromDegrees(80.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketTwo() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(25.0, -148.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(175.0, -123.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(235.0, -98.0), Rotation2d.fromDegrees(275.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
