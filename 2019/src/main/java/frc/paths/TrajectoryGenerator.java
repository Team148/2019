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
    public static final Pose2d kLeftLevel1StartPose = new Pose2d(new Translation2d(66.0, 50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightLevel1StartPose = new Pose2d(new Translation2d(66.0, -50.0), Rotation2d.fromDegrees(0.0));

    //Level 2
    public static final Pose2d kLeftLevel2StartPose = new Pose2d(new Translation2d(18.0, 50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightLevel2StartPose = new Pose2d(new Translation2d(18.0, -50.0), Rotation2d.fromDegrees(0.0));
 
    //Off Level 2
    public static final Pose2d kLeftOffLevel2Pose = new Pose2d(new Translation2d(120.0, 50.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightOffLevel2Pose = new Pose2d(new Translation2d(120.0, -50.0), Rotation2d.fromDegrees(0.0));

    //Scoring Poses
    //Cargo One
    public static final Pose2d kLevel1LeftCargoOnePose = new Pose2d(new Translation2d(205.0, 20.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevel1RightCargoOnePose = new Pose2d(new Translation2d(205.0, -20.0), Rotation2d.fromDegrees(0.0));

    public static final Pose2d kLevel2LeftCargoOnePose = new Pose2d(new Translation2d(205.0, 13.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLevel2RightCargoOnePose = new Pose2d(new Translation2d(205.0, -11.0), Rotation2d.fromDegrees(0.0));

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

    //Rocket One Stop
    public static final Pose2d kLevel1LeftRocketOneStopPose = new Pose2d(new Translation2d(200.0-15.8, 138.0-8.6), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kLevel1RightRocketOneStopPose = new Pose2d(new Translation2d(200.0-15.8, -138.0+8.6), Rotation2d.fromDegrees(330.0));

    public static final Pose2d kLevel2LeftRocketOneStopPose = new Pose2d(new Translation2d(195.0-15.8, 130.0-8.6), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kLevel2RightRocketOneStopPose = new Pose2d(new Translation2d(195.0-15.8, -130.0+8.6), Rotation2d.fromDegrees(330.0));


    //Rocket One
    public static final Pose2d kLevel1LeftRocketOnePose = new Pose2d(new Translation2d(200.0, 138.0), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kLevel1RightRocketOnePose = new Pose2d(new Translation2d(200.0, -138.0), Rotation2d.fromDegrees(330.0));

    

    public static final Pose2d kLevel2LeftRocketOnePose = new Pose2d(new Translation2d(195.0, 130.0), Rotation2d.fromDegrees(30.0));
    // public static final Pose2d kLevel2RightRocketOnePose = new Pose2d(new Translation2d(200.0, -139.0), Rotation2d.fromDegrees(315.0));
    public static final Pose2d kLevel2RightRocketOnePose = new Pose2d(new Translation2d(195.0, -130.0), Rotation2d.fromDegrees(330.0));

    //Rocket One Stop Plus Minus

    public static final Pose2d kRightRocketOneStopRocketOneMinusOnePose = kLevel1RightRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(0.96,-1.75)));
    public static final Pose2d kRightRocketOneStopRocketOnePlusOnePose = kLevel1RightRocketOnePose.transformBy(Pose2d.fromTranslation(new Translation2d(-0.96,1.75)));




    //Rocket Two
    public static final Pose2d kLeftRocketTwoPose = new Pose2d(new Translation2d(228.0, 115.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kRightRocketTwoPose = new Pose2d(new Translation2d(228.0, -115.0), Rotation2d.fromDegrees(270.0));

    //Rocket Three  
    public static final Pose2d kLeftRocketThreePose = new Pose2d(new Translation2d(260.0, 125.0), Rotation2d.fromDegrees(150.0));
    public static final Pose2d kRightRocketThreePose = new Pose2d(new Translation2d(260.0, -125.0), Rotation2d.fromDegrees(210.0));

    public static final Pose2d kLevel2RocketThreePose = new Pose2d(new Translation2d(280.0, 125.0), Rotation2d.fromDegrees(150.0));
    // public static final Pose2d kLevel2RocketThreePose = new Pose2d(new Translation2d(280.0, -125.0), Rotation2d.fromDegrees(210.0));

    //Loading Station Poses
    public static final Pose2d kLeftLoadingStationPose = new Pose2d(new Translation2d(25.0, 132.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kRightLoadingStationPose = new Pose2d(new Translation2d(25.0, -132.0), Rotation2d.fromDegrees(180.0)); 

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
        public final MirroredTrajectory straightTest;

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

        public final Trajectory<TimedState<Pose2dWithCurvature>> rightLevel1toCargoOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightLevel2toCargoOne;

        public final Trajectory<TimedState<Pose2dWithCurvature>> leftLevel1toCargoOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftLevel2toCargoOne;

        public final Trajectory<TimedState<Pose2dWithCurvature>> rightLevel1toRocketOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightLevel2toRocketOne;

        public final Trajectory<TimedState<Pose2dWithCurvature>> leftLevel1toRocketOne;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftLevel2toRocketOne;

        public final Trajectory<TimedState<Pose2dWithCurvature>> leftRocketOneToTurn;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightRocketOneToTurn;

        public final Trajectory<TimedState<Pose2dWithCurvature>> endLeftTurnToLoadingStation;
        public final Trajectory<TimedState<Pose2dWithCurvature>> endRightTurnToLoadingStation;

        public final Trajectory<TimedState<Pose2dWithCurvature>> rightLevel1toRocketOneStop;
        public final Trajectory<TimedState<Pose2dWithCurvature>> RocketOneStoptoRocketOneMinus1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> RocketOneStoptoRocketOnePlus1;

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

            straightTest = new MirroredTrajectory(getStraightTest());

            rightLevel1toCargoOne = getRightLevel1toCargoOne();
            rightLevel2toCargoOne = getRightLevel2toCargoOne();

            leftLevel1toCargoOne = getLeftLevel1toCargoOne();
            leftLevel2toCargoOne = getLeftLevel2toCargoOne();

            rightLevel1toRocketOne = getRightLevel1toRocketOne();
            rightLevel2toRocketOne = getRightLevel2toRocketOne();

            leftLevel1toRocketOne = getLeftLevel1toRocketOne();
            leftLevel2toRocketOne = getLeftLevel2toRocketOne();

            rightRocketOneToTurn = getRightRocketOneToTurn();
            leftRocketOneToTurn = getLeftRocketOneToTurn();

            endRightTurnToLoadingStation = getEndRightTurnToLoadStation();
            endLeftTurnToLoadingStation = getEndLeftTurnToLoadStation();


            rightLevel1toRocketOneStop =getRightLevel1toRocketOneStop;
            RocketOneStoptoRocketOneMinus1 = getRightRocketOneStoptoRocketOneMinus1;
            RocketOneStoptoRocketOnePlus1 = getRightRocketOneStoptoRocketOnePlus1;

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

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightLevel1toRocketOneStop() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLevel1StartPose);
            waypoints.add(kLevel1RightRocketOneStopPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightRocketOneStoptoRocketOneMinus1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel1RightRocketOneStopPose);
            waypoints.add(kRightRocketOneStopRocketOneMinusOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
                kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRightRocketOneStoptoRocketOnePlus1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLevel1RightRocketOneStopPose);
            waypoints.add(kRightRocketOneStopRocketOnePlusOnePose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)),
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
