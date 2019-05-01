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

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    // private static final double kMaxCentripetalAccelElevatorDown = 30.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    // private static final double kFirstPathMaxVoltage = 9.0;
    // private static final double kFirstPathMaxAccel = 130.0;
    // private static final double kFirstPathMaxVel = 130.0;
    // private static final double kSimpleSwitchMaxAccel = 100.0;
    // private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    // private static final double kSimpleSwitchMaxVelocity = 120.0;

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
    public static final Pose2d kLevelOneStartPoseLeft = new Pose2d(new Translation2d(66.0, 50.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLevelOneStartBackwardPose = new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLevelOneStartBackwardPoseLeft = new Pose2d(new Translation2d(66.0, 50.0), Rotation2d.fromDegrees(180.0));

    // //Level 2
    public static final Pose2d kLevelTwoStartPose = new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevelTwoStartPoseLeft = new Pose2d(new Translation2d(18.0, 50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevelTwoStartBackwardPose = new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLevelTwoStartBackwardPoseLeft = new Pose2d(new Translation2d(18.0, 50.0), Rotation2d.fromDegrees(180.0));
 
    // //Off Level 2
    public static final Pose2d kOffHabPlatform = new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kOffHabPlatformLeft = new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kOffHabPlatformBackwards = new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kOffHabPlatformBackwardsLeft = new Pose2d(new Translation2d(100.0, 50.0), Rotation2d.fromDegrees(180.0));

    // //Scoring Poses
    // //Cargo One
    public static final Pose2d kCargoOnePose = new Pose2d(new Translation2d(205.0, -11.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCargoOnePoseLeft = new Pose2d(new Translation2d(205.0, 11.0), Rotation2d.fromDegrees(0.0));

    // // Opposite Side Cargo One
    public static final Pose2d kOppositeCargoOnePose = new Pose2d(new Translation2d(205.0, 11.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kOppositeCargoOnePoseLeft = new Pose2d(new Translation2d(205.0, -11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two
    // public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(261.0, -40.0), Rotation2d.fromDegrees(90.0));
    // public static final Pose2d kCargoTwoPoseLeft = new Pose2d(new Translation2d(261.0, 40.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(265.0, -40.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoPoseLeft = new Pose2d(new Translation2d(265.0, 40.0), Rotation2d.fromDegrees(270.0));

    // //Cargo Three
    public static final Pose2d kCargoThreePose = new Pose2d(new Translation2d(281.0, -40.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoThreePoseLeft = new Pose2d(new Translation2d(281.0, 40.0), Rotation2d.fromDegrees(270.0));

    // //Cargo Four
    public static final Pose2d kCargoFourPose = new Pose2d(new Translation2d(303.0, -40.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoFourPoseLeft = new Pose2d(new Translation2d(303.0, 40.0), Rotation2d.fromDegrees(270.0));

    // //Rocket One
    public static final Pose2d kRocketOnePose = new Pose2d(new Translation2d(197.0, -133.0), Rotation2d.fromDegrees(330.0));
    public static final Pose2d kRocketOnePoseLeft = new Pose2d(new Translation2d(197.0, 133.0), Rotation2d.fromDegrees(30.0));

    // //Rocket Three
    public static final Pose2d kRocketThreePose = new Pose2d(new Translation2d(260.0, -135.0), Rotation2d.fromDegrees(210.0));
    public static final Pose2d kRocketThreePoseLeft = new Pose2d(new Translation2d(260.0, 135.0), Rotation2d.fromDegrees(150.0));

    // //Loading Station Poses
    public static final Pose2d kLoadingStationPose = new Pose2d(new Translation2d(23.0, -138.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLoadingStationPoseLeft = new Pose2d(new Translation2d(23.0, 138.0), Rotation2d.fromDegrees(180.0)); 

    // //Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kCargoOneLineupPose = new Pose2d(new Translation2d(165.0, -10.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kCargoOneLineupPoseLeft = new Pose2d(new Translation2d(165.0, 12.0), Rotation2d.fromDegrees(0.0));
    // public static final Pose2d kCargoOneLineupPose = new Pose2d(new Translation2d(175.0, -11.0), Rotation2d.fromDegrees(0.0));
    // public static final Pose2d kCargoOneLineupPoseLeft = new Pose2d(new Translation2d(175.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Opposite Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kOppositeCargoOneLineupPose = new Pose2d(new Translation2d(175.0, 11.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kOppositeCargoOneLineupPoseLeft = new Pose2d(new Translation2d(175.0, -11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two Lineup (for Vision Tracking)
    // public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(261.0, -80.0), Rotation2d.fromDegrees(90.0));
    // public static final Pose2d kCargoTwoLineupPoseLeft = new Pose2d(new Translation2d(261.0, 80.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(260.0, -65.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoLineupPoseLeft = new Pose2d(new Translation2d(260.0, 65.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kCargoTwoBackupPose = new Pose2d(new Translation2d(261.0, -100.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoBackupPoseLeft = new Pose2d(new Translation2d(261.0, 100.0), Rotation2d.fromDegrees(270.0));

    public static final Pose2d kLoadingToCargoTwoLineupPose = new Pose2d(new Translation2d(265.0, -75.0), Rotation2d.fromDegrees(200.0));
    public static final Pose2d kLoadingToCargoTwoLineupPoseLeft = new Pose2d(new Translation2d(265.0, 75.0), Rotation2d.fromDegrees(160.0));

    // //Cargo Three Lineup (for Vision Tracking)
    public static final Pose2d kCargoThreeLineupPose = new Pose2d(new Translation2d(280.0, -80.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoThreeLineupPoseLeft = new Pose2d(new Translation2d(280.0, 80.0), Rotation2d.fromDegrees(270.0));

    // //Cargo Four Lineup (for Vision Tracking)
    public static final Pose2d kCargoFourLineupPose = new Pose2d(new Translation2d(303.0, -70.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoFourLineupPoseLeft = new Pose2d(new Translation2d(303.0, 70.0), Rotation2d.fromDegrees(270.0));

    // //Rocket One Lineup (for Vision Tracking)
    public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(171.0, -119.0), Rotation2d.fromDegrees(330.0));
    public static final Pose2d kRocketOneLineupPoseLeft = new Pose2d(new Translation2d(171.0, 119.0), Rotation2d.fromDegrees(30.0));

    // //Rocket Three Lineup (for Vision Tracking)
    // public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -110.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(300.0, -118.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRocketThreeLineupPoseLeft = new Pose2d(new Translation2d(300.0, 125.0), Rotation2d.fromDegrees(180.0));

    // //Loading Station Lineup (for Vision Tracking)
    // public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(40.0, -135.0), Rotation2d.fromDegrees(180.0));
    // public static final Pose2d kLoadingStationLineupPoseLeft = new Pose2d(new Translation2d(40.0, 135.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(40.0, -138.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLoadingStationLineupPoseLeft = new Pose2d(new Translation2d(40.0, 130.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kComingToLoadingStation = kLoadingStationLineupPose.transformBy(Pose2d.fromTranslation(new Translation2d(30.0, 0.0)));
    public static final Pose2d kComingToLoadingStationLeft = kLoadingStationLineupPoseLeft.transformBy(Pose2d.fromTranslation(new Translation2d(30.0, 0.0)));
    
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

        // public final MirroredTrajectory goStraight;

        //Lineup to First Goal Trajectories
        // public final MirroredTrajectory levelOneToCargoTwoLineup;
        // public final MirroredTrajectory levelTwoToCargoTwoLineup;

        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToCargoOneLineupForwardRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToCargoOneLineupForwardLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToCargoOneLineupForwardRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToCargoOneLineupForwardLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToCargoTwoLineupForwardRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToCargoTwoLineupForwardLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToCargoTwoLineupForwardRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToCargoTwoLineupForwardLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToRocketOneLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelOneToRocketOneLineupLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToRocketOneLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> levelTwoToRocketOneLineupLeft;
        
        // public final MirroredTrajectory levelOneToRocketThreeLineup;
        // public final MirroredTrajectory levelTwoToRocketThreeLineup;
        // public final MirroredTrajectory cargoTwoLineupToCargoTwo;
        // public final MirroredTrajectory cargoThreeLineupToCargoThree;
        // public final MirroredTrajectory rocketOneLineupToRocketOne;
        // public final MirroredTrajectory rocketThreeLineupToRocketThree;
        // public final MirroredTrajectory loadingStationLineupToLoadingStation;

        // public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromCargoTwoRight;
        // public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromCargoTwoLeft;

        // public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromRocketOneRight;
        // public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromRocketOneLeft;

        // public final MirroredTrajectory awayFromRocketThree;

        public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromRocketOneToLoadingStationLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> awayFromRocketOneToLoadingStationLineupLeft;

        // public final MirroredTrajectory rocketThreeLineupToLoadingStationLineup;

        // public final MirroredTrajectory cargoTwoLineupToLoadingStation;

        public final Trajectory<TimedState<Pose2dWithCurvature>> endCargoOneToLoadingStatonRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> endCargoOneToLoadingStatonLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> endCargoTwoToLoadingStatonRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> endCargoTwoToLoadingStatonLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToCargoTwoLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToCargoTwoLineupLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToCargoThreeLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToCargoThreeLineupLeft;

        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToRocketThreeLineupRight;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadingStationToRocketThreeLineupLeft;


        private TrajectorySet() {

            // goStraight = new MirroredTrajectory(getGoStraight());

            // levelOneToCargoTwoLineup = new MirroredTrajectory(getLevelOneToCargoTwoLineup());
            // levelTwoToCargoTwoLineup = new MirroredTrajectory(getLevelTwoToCargoTwoLineup());

            levelOneToCargoOneLineupForwardRight = getLevelOneToCargoOneLineupForwardRight();
            levelOneToCargoOneLineupForwardLeft = getLevelOneToCargoOneLineupForwardLeft();
            levelTwoToCargoOneLineupForwardRight = getLevelTwoToCargoOneLineupForwardRight();
            levelTwoToCargoOneLineupForwardLeft = getLevelTwoToCargoOneLineupForwardLeft();

            levelOneToCargoTwoLineupForwardRight = getLevelOneToCargoTwoLineupForwardRight();
            levelOneToCargoTwoLineupForwardLeft = getLevelOneToCargoTwoLineupForwardLeft();
            levelTwoToCargoTwoLineupForwardRight = getLevelTwoToCargoTwoLineupForwardRight();
            levelTwoToCargoTwoLineupForwardLeft = getLevelTwoToCargoTwoLineupForwardLeft();

            levelOneToRocketOneLineupRight = getLevelOneToRocketOneLineupRight();
            levelOneToRocketOneLineupLeft = getLevelOneToRocketOneLineupLeft();
            levelTwoToRocketOneLineupRight = getLevelTwoToRocketOneLineupRight();
            levelTwoToRocketOneLineupLeft = getLevelTwoToRocketOneLineupLeft();

            // levelOneToRocketThreeLineup = new MirroredTrajectory(getLevelOneToRocketThreeLineup());
            // levelTwoToRocketThreeLineup = new MirroredTrajectory(getLevelTwoToRocketThreeLineup());
            // cargoTwoLineupToCargoTwo = new MirroredTrajectory(getCargoTwoLineupToCargoTwo());
            // cargoThreeLineupToCargoThree = new MirroredTrajectory(getCargoThreeLineupToCargoThree());
            // rocketOneLineupToRocketOne = new MirroredTrajectory(getRocketOneLineupToRocketOne());
            // rocketThreeLineupToRocketThree = new MirroredTrajectory(getRocketThreeLineupToRocketThree());
            // loadingStationLineupToLoadingStation = new MirroredTrajectory(getLoadingStationLineupToLoadingStation());

            // awayFromCargoTwoRight = getAwayFromCargoTwoRight();
            // awayFromCargoTwoLeft = getAwayFromCargoTwoLeft();

            // awayFromRocketOneRight = getAwayFromRocketOneRight();
            // awayFromRocketOneLeft = getAwayFromRocketOneLeft();

            // awayFromRocketThree = new MirroredTrajectory(getAwayFromRocketThree());

            // cargoTwoLineupToLoadingStation = new MirroredTrajectory(getCargoTwoLineupToLoadingStation());

            endCargoOneToLoadingStatonRight = getEndCargoOneToLoadingStationRight();
            endCargoOneToLoadingStatonLeft = getEndCargoOneToLoadingStationLeft();

            endCargoTwoToLoadingStatonRight = getEndCargoTwoToLoadingStationRight();
            endCargoTwoToLoadingStatonLeft = getEndCargoTwoToLoadingStationLeft();

            awayFromRocketOneToLoadingStationLineupRight = getAwayFromRocketOneToLoadingStationLineupRight();
            awayFromRocketOneToLoadingStationLineupLeft = getAwayFromRocketOneToLoadingStationLineupLeft();

            // rocketThreeLineupToLoadingStationLineup = new MirroredTrajectory(getRocketThreeLineupToLoadingStationLineup());

            loadingStationToCargoTwoLineupRight = getLoadingStationToCargoTwoLineupRight();
            loadingStationToCargoTwoLineupLeft = getLoadingStationToCargoTwoLineupLeft();

            loadingStationToCargoThreeLineupRight = getLoadingStationToCargoThreeLineupRight();
            loadingStationToCargoThreeLineupLeft = getLoadingStationToCargoThreeLineupLeft();

            loadingStationToRocketThreeLineupRight = getLoadingStationToRocketThreeLineupRight();
            loadingStationToRocketThreeLineupLeft = getLoadingStationToRocketThreeLineupLeft();
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getGoStraight() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
        //     waypoints.add(new Pose2d(new Translation2d(200.0, -50.0), Rotation2d.fromDegrees(0.0)));

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoOneLineupForwardRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(165.0, -10.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(205.0, -10.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoOneLineupForwardLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(new Pose2d(new Translation2d(165.0, 12.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(205.0, 12.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToCargoOneLineupForwardRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(165.0, -10.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(205.0, -10.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToCargoOneLineupForwardLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(new Pose2d(new Translation2d(165.0, 10.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(205.0, 10.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoTwoLineupForwardRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(224.0, -100.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -80.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -70.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -40.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoTwoLineupForwardLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(new Pose2d(new Translation2d(224.0, 100.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 80.0), Rotation2d.fromDegrees(270.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 70.0), Rotation2d.fromDegrees(270.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 40.0), Rotation2d.fromDegrees(270.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToCargoTwoLineupForwardRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(224.0, -100.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -80.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -70.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, -40.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToCargoTwoLineupForwardLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(new Pose2d(new Translation2d(224.0, 100.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 80.0), Rotation2d.fromDegrees(270.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 70.0), Rotation2d.fromDegrees(270.0)));
            waypoints.add(new Pose2d(new Translation2d(264.0, 40.0), Rotation2d.fromDegrees(270.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToRocketOneLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToRocketOneLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(kRocketOneLineupPoseLeft);
            waypoints.add(kRocketOnePoseLeft);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(80.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToRocketOneLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(kRocketOneLineupPose);
            waypoints.add(kRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToRocketOneLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelTwoStartPoseLeft);
            waypoints.add(kOffHabPlatformLeft);
            waypoints.add(kRocketOneLineupPoseLeft);
            waypoints.add(kRocketOnePoseLeft);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToRocketThreeLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevelOneStartBackwardPose);
        //     waypoints.add(kOffHabPlatformBackwards);
        //     waypoints.add(new Pose2d(new Translation2d(250.0, -115.0), Rotation2d.fromDegrees(150.0)));
        //     waypoints.add(kRocketThreeLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getLevelTwoToRocketThreeLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLevelOneStartPose);
        //     waypoints.add(kOffHabPlatform);
        //     waypoints.add(new Pose2d(new Translation2d(220.0, -100.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(kRocketThreeLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoLineupToCargoTwo() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoTwoLineupPose);
        //     waypoints.add(kCargoTwoPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoThreeLineupToCargoThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoThreeLineupPose);
        //     waypoints.add(kCargoThreePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketOneLineupToRocketOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRocketOneLineupPose);
        //     waypoints.add(kRocketOnePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeLineupToRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRocketThreeLineupPose);
        //     waypoints.add(kRocketThreePose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }
        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationLineupToLoadingStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLoadingStationLineupPose);
        //     waypoints.add(kLoadingStationPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        //retreat to lineup waypoint
        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromOppositeCargoOne() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kOppositeCargoOnePose);
        //     waypoints.add(kOppositeCargoOneLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromCargoTwoRight() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(261.0, -30.0), Rotation2d.fromDegrees(90.0)));
        //     waypoints.add(new Pose2d(new Translation2d(261.0, -100.0), Rotation2d.fromDegrees(90.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(285.0, -60.0), Rotation2d.fromDegrees(180.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(295.0, -60.0), Rotation2d.fromDegrees(180.0)));
        //     // waypoints.add(kCargoTwoPose);
        //     // waypoints.add(kCargoTwoBackupPose);
        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromCargoTwoLeft() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(261.0, 30.0), Rotation2d.fromDegrees(270.0)));
        //     waypoints.add(new Pose2d(new Translation2d(261.0, 100.0), Rotation2d.fromDegrees(270.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(285.0, -60.0), Rotation2d.fromDegrees(180.0)));
        //     // waypoints.add(new Pose2d(new Translation2d(295.0, -60.0), Rotation2d.fromDegrees(180.0)));
        //     // waypoints.add(kCargoTwoPose);
        //     // waypoints.add(kCargoTwoBackupPose);
        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getCargoTwoLineupToLoadingStation() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     // waypoints.add(kCargoTwoLineupPose);
        //     waypoints.add(kCargoTwoBackupPose);
        //     waypoints.add(new Pose2d(new Translation2d(200.0, -55.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(kComingToLoadingStation);
        //     waypoints.add(kLoadingStationLineupPose);
        //     // waypoints.add(kLoadingStationPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
        //         0.0, 20.0, 50.0, 50.0, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndCargoOneToLoadingStationRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(180.0, -10.0), Rotation2d.fromDegrees(240.0)));
            waypoints.add(new Pose2d(new Translation2d(40.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(18.0, -138.0), Rotation2d.fromDegrees(181.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(70.0)),
                0.0, 30.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndCargoOneToLoadingStationLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(180.0, 10.0), Rotation2d.fromDegrees(120.0)));
            waypoints.add(new Pose2d(new Translation2d(40.0, 130.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(18.0, 130.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(70.0)),
                0.0, 30.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndCargoTwoToLoadingStationRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(264.0, -60.0), Rotation2d.fromDegrees(210.0)));
            waypoints.add(new Pose2d(new Translation2d(125.0, -130.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(100.0, -130.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(40.0, -130.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 30.0, 50.0, 50.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndCargoTwoToLoadingStationLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(264.0, 60.0), Rotation2d.fromDegrees(150.0)));
            waypoints.add(new Pose2d(new Translation2d(125.0, 130.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(100.0, 130.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(40.0, 130.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 30.0, 50.0, 50.0, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromCargoThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kCargoThreePose);
        //     waypoints.add(kCargoThreeLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOneRight() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRocketOnePose);
        //     waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOneLeft() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRocketOnePoseLeft);
        //     waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketThree() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kRocketThreePose);
        //     waypoints.add(new Pose2d(new Translation2d(289.0, -119.0), Rotation2d.fromDegrees(180.0)));

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

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

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOneToLoadingStationLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(165.0, -90.0), Rotation2d.fromDegrees(250.0)));
            waypoints.add(new Pose2d(new Translation2d(90.0, -133.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kLoadingStationLineupPose);
            waypoints.add(new Pose2d(new Translation2d(60.0, -133.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAwayFromRocketOneToLoadingStationLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(165.0, 90.0), Rotation2d.fromDegrees(110.0)));
            waypoints.add(new Pose2d(new Translation2d(90.0, 133.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kLoadingStationLineupPose);
            waypoints.add(new Pose2d(new Translation2d(60.0, 133.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getRocketThreeLineupToLoadingStationLineup() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(new Pose2d(new Translation2d(289.0, -119.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(new Pose2d(new Translation2d(200.0, -115.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(kLoadingStationLineupPose);

        //     return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         kMaxVelocity, kMaxAccel, kMaxVoltage);
        // }

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

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoTwoLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            // waypoints.add(new Pose2d(new Translation2d(100.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(270.0, -75.0), Rotation2d.fromDegrees(200.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoTwoLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPoseLeft);
            // waypoints.add(new Pose2d(new Translation2d(100.0, 138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(270.0, 75.0), Rotation2d.fromDegrees(160.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoThreeLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(125.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(239.0, -100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, -120.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, -130.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToCargoThreeLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(new Pose2d(new Translation2d(100.0, 138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(125.0, 138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(239.0, 100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, 120.0), Rotation2d.fromDegrees(270.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, 130.0), Rotation2d.fromDegrees(270.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, 80.0, kMaxVoltage);
        }

        // private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketThreeLineupRight() {
        //     List<Pose2d> waypoints = new ArrayList<>();
        //     waypoints.add(kLoadingStationPose);
        //     waypoints.add(new Pose2d(new Translation2d(230.0, -110.0), Rotation2d.fromDegrees(180.0)));
        //     waypoints.add(kRocketThreeLineupPose);

        //     return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
        //         80.0, 80.0, kMaxVoltage);
        // }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketThreeLineupRight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(new Pose2d(new Translation2d(230.0, -118.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketThreeLineupLeft() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPoseLeft);
            waypoints.add(new Pose2d(new Translation2d(230.0, 125.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kRocketThreeLineupPoseLeft);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}
