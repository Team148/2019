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
    private static final double kMaxVelocity = 130.0;
    private static final double kMaxAccel = 130.0;
    // private static final double kMaxVelocity = 120.0;
    // private static final double kMaxAccel = 80.0;
    // private static final double kMaxAccel = 100.0;
    // private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    // private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxCentripetalAccelElevatorDown = 30.0;
    // private static final double kMaxCentripetalAccel = 50.0;
    private static final double kMaxCentripetalAccel = 100.0;
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
    // public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(253.0, -40.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoPose = new Pose2d(new Translation2d(261.0, -40.0), Rotation2d.fromDegrees(90.0));

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
    public static final Pose2d kLoadingStationPose = new Pose2d(new Translation2d(23.0, -138.0), Rotation2d.fromDegrees(180.0)); 

    // //Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kCargoOneLineupPose = new Pose2d(new Translation2d(175.0, -11.0), Rotation2d.fromDegrees(0.0));

    // //Opposite Cargo One Lineup (for Vision Tracking)
    public static final Pose2d kOppositeCargoOneLineupPose = new Pose2d(new Translation2d(175.0, 11.0), Rotation2d.fromDegrees(0.0));

    // //Cargo Two Lineup (for Vision Tracking)
    public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(261.0, -80.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kCargoTwoBackupPose = new Pose2d(new Translation2d(261.0, -100.0), Rotation2d.fromDegrees(90.0));
    // public static final Pose2d kCargoTwoLineupPose = new Pose2d(new Translation2d(253.0, -80.0), Rotation2d.fromDegrees(90.0));
    // public static final Pose2d kCargoTwoBackupPose = new Pose2d(new Translation2d(253.0, -100.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Three Lineup (for Vision Tracking)
    public static final Pose2d kCargoThreeLineupPose = new Pose2d(new Translation2d(280.0, -80.0), Rotation2d.fromDegrees(90.0));

    // //Cargo Four Lineup (for Vision Tracking)
    public static final Pose2d kCargoFourLineupPose = new Pose2d(new Translation2d(303.0, -70.0), Rotation2d.fromDegrees(90.0));

    // //Rocket One Lineup (for Vision Tracking)
    public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(171.0, -119.0), Rotation2d.fromDegrees(330.0));
    // public static final Pose2d kRocketOneLineupPose = new Pose2d(new Translation2d(178.0, -125.0), Rotation2d.fromDegrees(330.0));

    // //Rocket Three Lineup (for Vision Tracking)
    public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -119.0), Rotation2d.fromDegrees(135.0));
    // public static final Pose2d kRocketThreeLineupPose = new Pose2d(new Translation2d(286.0, -119.0), Rotation2d.fromDegrees(180.0));

    // //Loading Station Lineup (for Vision Tracking)
    // public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(50.0, -138.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kLoadingStationLineupPose = new Pose2d(new Translation2d(40.0, -135.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kComingToLoadingStation = kLoadingStationLineupPose.transformBy(Pose2d.fromTranslation(new Translation2d(30.0, 0.0)));
    
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

        public final MirroredTrajectory goStraight;

        //Lineup to First Goal Trajectories
        public final MirroredTrajectory levelOneToCargoTwoLineup;
        public final MirroredTrajectory levelTwoToCargoTwoLineup;

        public final MirroredTrajectory levelOneToCargoTwoLineupForward;

        public final MirroredTrajectory levelOneToRocketOneLineup;
        public final MirroredTrajectory levelTwoToRocketOneLineup;
        
        public final MirroredTrajectory levelOneToRocketThreeLineup;
        public final MirroredTrajectory levelTwoToRocketThreeLineup;

        public final MirroredTrajectory cargoTwoLineupToCargoTwo;
        public final MirroredTrajectory cargoThreeLineupToCargoThree;

        public final MirroredTrajectory rocketOneLineupToRocketOne;
        public final MirroredTrajectory rocketThreeLineupToRocketThree;
        public final MirroredTrajectory loadingStationLineupToLoadingStation;

        public final MirroredTrajectory awayFromCargoTwo;
        public final MirroredTrajectory awayFromRocketOne;
        public final MirroredTrajectory awayFromRocketThree;

        public final MirroredTrajectory awayFromRocketOneToLoadingStationLineup;
        public final MirroredTrajectory rocketThreeLineupToLoadingStationLineup;

        public final MirroredTrajectory cargoTwoLineupToLoadingStation;
        public final MirroredTrajectory endCargoTwoToLoadingStaton;

        public final MirroredTrajectory loadingStationToCargoThreeLineup;
        public final MirroredTrajectory loadingStationToRocketThreeLineup;

        private TrajectorySet() {

            goStraight = new MirroredTrajectory(getGoStraight());

            levelOneToCargoTwoLineup = new MirroredTrajectory(getLevelOneToCargoTwoLineup());
            levelTwoToCargoTwoLineup = new MirroredTrajectory(getLevelTwoToCargoTwoLineup());

            levelOneToCargoTwoLineupForward = new MirroredTrajectory(getLevelOneToCargoTwoLineupForward());

            levelOneToRocketOneLineup = new MirroredTrajectory(getLevelOneToRocketOneLineup());
            levelTwoToRocketOneLineup = new MirroredTrajectory(getLevelTwoToRocketOneLineup());

            levelOneToRocketThreeLineup = new MirroredTrajectory(getLevelOneToRocketThreeLineup());
            levelTwoToRocketThreeLineup = new MirroredTrajectory(getLevelTwoToRocketThreeLineup());

            cargoTwoLineupToCargoTwo = new MirroredTrajectory(getCargoTwoLineupToCargoTwo());
            cargoThreeLineupToCargoThree = new MirroredTrajectory(getCargoThreeLineupToCargoThree());

            rocketOneLineupToRocketOne = new MirroredTrajectory(getRocketOneLineupToRocketOne());
            rocketThreeLineupToRocketThree = new MirroredTrajectory(getRocketThreeLineupToRocketThree());
            loadingStationLineupToLoadingStation = new MirroredTrajectory(getLoadingStationLineupToLoadingStation());

            awayFromCargoTwo = new MirroredTrajectory(getAwayFromCargoTwo());

            awayFromRocketOne = new MirroredTrajectory(getAwayFromRocketOne());
            awayFromRocketThree = new MirroredTrajectory(getAwayFromRocketThree());

            cargoTwoLineupToLoadingStation = new MirroredTrajectory(getCargoTwoLineupToLoadingStation());
            endCargoTwoToLoadingStaton = new MirroredTrajectory(getEndCargoTwoToLoadingStation());

            awayFromRocketOneToLoadingStationLineup = new MirroredTrajectory(getAwayFromRocketOneToLoadingStationLineup());
            rocketThreeLineupToLoadingStationLineup = new MirroredTrajectory(getRocketThreeLineupToLoadingStationLineup());

            loadingStationToCargoThreeLineup = new MirroredTrajectory(getLoadingStationToCargoThreeLineup());
            loadingStationToRocketThreeLineup = new MirroredTrajectory(getLoadingStationToRocketThreeLineup());
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGoStraight() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(100.0, -50.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(200.0, -50.0), Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoTwoLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            // waypoints.add(kLevelOneStartBackwardPose);
            // waypoints.add(kOffHabPlatformBackwards);
            // waypoints.add(new Pose2d(new Translation2d(225.0, -55.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kCargoTwoLineupPose);
            waypoints.add(kLevelOneStartBackwardPose);
            waypoints.add(kOffHabPlatformBackwards);
            waypoints.add(new Pose2d(new Translation2d(200.0, -60.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kCargoTwoLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevelOneToCargoTwoLineupForward() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevelOneStartPose);
            waypoints.add(kOffHabPlatform);
            waypoints.add(new Pose2d(new Translation2d(209.0, -100.0), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(new Translation2d(249.0, -80.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(249.0, -70.0), Rotation2d.fromDegrees(90.0)));
            // waypoints.add(kCargoTwoLineupPose);
            // waypoints.add(new Pose2d(new Translation2d(261.0, -50.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
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

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                0.0, 20.0, kMaxVelocity, 80.0, kMaxVoltage);
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
            waypoints.add(new Pose2d(new Translation2d(261.0, -30.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(261.0, -100.0), Rotation2d.fromDegrees(90.0)));
            // waypoints.add(new Pose2d(new Translation2d(285.0, -60.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(295.0, -60.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kCargoTwoPose);
            // waypoints.add(kCargoTwoBackupPose);
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

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(50.0)),
                0.0, 20.0, 50.0, 50.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getEndCargoTwoToLoadingStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(261.0, -100.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(225.0, -60.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(90.0, -112.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(70.0, -112.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(50.0, -112.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(295.0, -60.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(125.0, -138.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(40.0, -138.0), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
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
            waypoints.add(new Pose2d(new Translation2d(90.0, -128.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(kLoadingStationLineupPose);
            waypoints.add(new Pose2d(new Translation2d(60.0, -128.0), Rotation2d.fromDegrees(180.0)));

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
            waypoints.add(new Pose2d(new Translation2d(100.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(125.0, -138.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(239.0, -100.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, -120.0), Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(new Translation2d(269.0, -130.0), Rotation2d.fromDegrees(90.0)));
            // waypoints.add(new Pose2d(new Translation2d(245.0, -80.0), Rotation2d.fromDegrees(180.0)));
            // waypoints.add(new Pose2d(new Translation2d(278.0, -100.0), Rotation2d.fromDegrees(90.0)));
            // waypoints.add(new Pose2d(new Translation2d(278.0, -110.0), Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, 80.0, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadingStationToRocketThreeLineup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadingStationPose);
            waypoints.add(new Pose2d(new Translation2d(230.0, -90.0), Rotation2d.fromDegrees(180.0)));
            waypoints.add(kRocketThreeLineupPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                80.0, 80.0, kMaxVoltage);
        }
    }
}
