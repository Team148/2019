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
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
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
    public static final Pose2d kLeftLevel1StartPose = new Pose2d(new Translation2d(63.0, 45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightLevel1StartPose = new Pose2d(new Translation2d(63.0, -45.0), Rotation2d.fromDegrees(0.0));

    //Level 2
    public static final Pose2d kLeftLevel2StartPose = new Pose2d(new Translation2d(16.0, 45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightLevel2StartPose = new Pose2d(new Translation2d(16.0, -45.0), Rotation2d.fromDegrees(0.0));

    //Off Level 2
    public static final Pose2d kLeftOffLevel2Pose = new Pose2d(new Translation2d(120.0, 45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightOffLevel2Pose = new Pose2d(new Translation2d(120.0, -45.0), Rotation2d.fromDegrees(0.0));

    //Scoring Poses
    //Cargo One
    public static final Pose2d kLeftCargoOnePose = new Pose2d(new Translation2d(205.0, 20.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightCargoOnePose = new Pose2d(new Translation2d(205.0, -20.0), Rotation2d.fromDegrees(0.0));

    //Cargo Two
    public static final Pose2d kLeftCargoTwoPose = new Pose2d(new Translation2d(259.0, 45.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kRightCargoTwoPose = new Pose2d(new Translation2d(259.0, -45.0), Rotation2d.fromDegrees(90.0));

    //Cargo Three
    public static final Pose2d kLeftCargoThreePose = new Pose2d(new Translation2d(281.0, 45.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kRightCargoThreePose = new Pose2d(new Translation2d(281.0, -45.0), Rotation2d.fromDegrees(90.0));

    //Cargo Four
    public static final Pose2d kLeftCargoFourPose = new Pose2d(new Translation2d(302.0, 45.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kRightCargoFourPose = new Pose2d(new Translation2d(302.0, -45.0), Rotation2d.fromDegrees(90.0));

    //Rocket One
    public static final Pose2d kLeftRocketOnePose = new Pose2d(new Translation2d(202.0, 138.0), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kRightRocketOnePose = new Pose2d(new Translation2d(202.0, -138.0), Rotation2d.fromDegrees(330.0));

    //Rocket Two
    public static final Pose2d kLeftRocketTwoPose = new Pose2d(new Translation2d(228.0, 115.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kRightRocketTwoPose = new Pose2d(new Translation2d(228.0, -115.0), Rotation2d.fromDegrees(270.0));

    //Rocket Three
    public static final Pose2d kLeftRocketThreePose = new Pose2d(new Translation2d(260.0, 125.0), Rotation2d.fromDegrees(150.0));
    public static final Pose2d kRightRocketThreePose = new Pose2d(new Translation2d(260.0, -125.0), Rotation2d.fromDegrees(210.0));

    public static final Pose2d kLevel2RocketThreePose = new Pose2d(new Translation2d(280.0, 125.0), Rotation2d.fromDegrees(150.0));
    // public static final Pose2d kLevel2RocketThreePose = new Pose2d(new Translation2d(280.0, -125.0), Rotation2d.fromDegrees(210.0));

    //Loading Station Poses
    public static final Pose2d kLeftLoadingStationPose = new Pose2d(new Translation2d(16.0, 140.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightLoadingStationPose = new Pose2d(new Translation2d(16.0, -140.0), Rotation2d.fromDegrees(0.0)); 

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

        //Score first game piece
        public final MirroredTrajectory level1ToCargoOne;
        public final MirroredTrajectory level2ToCargoOne;
        
        public final MirroredTrajectory level1ToCargoTwo;
        public final MirroredTrajectory level2ToCargoTwo;

        public final MirroredTrajectory level1ToCargoThree;
        public final MirroredTrajectory level2ToCargoThree;

        public final MirroredTrajectory level1ToCargoFour;
        public final MirroredTrajectory level2ToCargoFour;

        public final MirroredTrajectory level1ToRocketOne;
        public final MirroredTrajectory level2ToRocketOne;

        public final MirroredTrajectory level1ToRocketTwo;
        public final MirroredTrajectory level2ToRocketTwo;

        public final MirroredTrajectory level1ToRocketThree;
        public final MirroredTrajectory level2ToRocketThree;

        //Score second game piece
        public final MirroredTrajectory loadingStationToRocketTwo;

        public final MirroredTrajectory cargoThreeTo90Turn;
        public final MirroredTrajectory endTurnToLoadingStation;
        public final MirroredTrajectory loadingStationToCargoFour;

        public final MirroredTrajectory endRocketToTurn;
        public final MirroredTrajectory endRocketTurnToLoadingStation;

        private TrajectorySet() {

            //Score first game piece
            level1ToCargoOne = new MirroredTrajectory(getLevel1ToCargoOne());
            level2ToCargoOne = new MirroredTrajectory(getLevel2ToCargoOne());

            level1ToCargoTwo = new MirroredTrajectory(getLevel1ToCargoTwo());
            level2ToCargoTwo = new MirroredTrajectory(getLevel2ToCargoTwo());

            level1ToCargoThree = new MirroredTrajectory(getLevel1ToCargoThree());
            level2ToCargoThree = new MirroredTrajectory(getLevel2ToCargoThree());

            level1ToCargoFour = new MirroredTrajectory(getLevel1ToCargoFour());
            level2ToCargoFour = new MirroredTrajectory(getLevel2ToCargoFour());

            level1ToRocketOne = new MirroredTrajectory(getLevel1ToRocketOne());
            level2ToRocketOne = new MirroredTrajectory(getLevel2ToRocketOne());

            level1ToRocketTwo = new MirroredTrajectory(getLevel1ToRocketTwo());
            level2ToRocketTwo = new MirroredTrajectory(getLevel2ToRocketTwo());

            level1ToRocketThree = new MirroredTrajectory(getLevel1ToRocketThree());
            level2ToRocketThree = new MirroredTrajectory(getLevel2ToRocketThree());

            //Score second game piece
            loadingStationToRocketTwo = new MirroredTrajectory(getLoadingStationToRocketTwo());

            cargoThreeTo90Turn = new MirroredTrajectory(getCargoThreeTo90Turn());
            endTurnToLoadingStation = new MirroredTrajectory(getEndTurnToLoadingStation());
            loadingStationToCargoFour = new MirroredTrajectory(getLoadingStationToCargoFour());

            endRocketToTurn = new MirroredTrajectory(getRocketThreeToTurn());
            endRocketTurnToLoadingStation = new MirroredTrajectory(getEndRocketTurnToLoadingStation());
        }

        //Score first game piece trajectories

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
            waypoints.add(kRightRocketOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2ToRocketOne() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel2StartPose);
            waypoints.add(kRightRocketOnePose);

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
