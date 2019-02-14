/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {
    
    //Drivetrain Constants
    public static final int DRIVE_ENCODER_TPR = 5000;

    //Elevator Constants
    public static final int ELEVATOR_SOFT_LIMIT = 36500;
    public static final double ELEVATOR_UP_OUTPUT_PERCENT = 1.0;
    public static final double ELEVATOR_DOWN_OUTPUT_PERCENT = -0.4;

    public static final double ELEVATOR_F_UP = 0.1367;
    public static final double ELEVATOR_F_DOWN = ELEVATOR_F_UP + 0.03;
    public static final double ELEVATOR_ZERO_F = -0.0732;
    public static final double ELEVATOR_P = 0.05;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.50;

    public static final double ELEVATOR_F_VELOCITY = 0.26;
    public static final double ELEVATOR_P_VELOCITY = 0.7;
    public static final double ELEVATOR_I_VELOCITY = 0.0;
    public static final double ELEVATOR_D_VELOCITY = 4.0;
    public static final double ELEVATOR_AUX_F_VELOCITY = 0.1367;

    public static final double ELEVATOR_ZERO = 0.0;
    public static final double ELEVATOR_MIDDLE = 15000;
    public static final double ELEVATOR_HIGH = 30000;
    public static final double ELEVATOR_MAX_HEIGHT = 34000;

    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION = 3000.0;
    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND = 1000.0;

    public static final double ELEVATOR_ERROR_TOLERANCE = 2000.0;
    public static final double ELEVATOR_MANUAL_DEADBAND = 0.2;
    public static final double ELEVATOR_MANUAL_DPOS_SCALAR = 12000.0;

    //Output Percents
    public static final double ELEVATOR_OUTPUT_PERCENT = 1.0;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 24.9;
    public static final double kDriveWheelDiameterInches = 5.912;// * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 44.9;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 14.99;	//34.473 / 2.0;
    public static final double kCenterToRearBumperDistance = 19.48;		//34.473 / 2.0;
    public static final double kCenterToSideBumperDistance = 32.47 / 2.0;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.7;	//0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 7.7;	//10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // PID gains for elevator velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per revolution.
    public static final double kElevatorHighGearKp = 0.15;
    public static final double kElevatorHighGearKi = 0.0;//0.0;
    public static final double kElevatorHighGearKd = 4.0;
    public static final double kElevatorHighGearKf = 0.06; // lower speed:  0.08;
    public static final double kElevatorJogKp = 0.1;
    public static final double kElevatorJogKd = 3.0;
    public static final double kElevatorFeedforwardNoCube = -0.06;//33000;
    public static final double kElevatorFeedforwardWithCube = -0.07;//33000;

    public static final int kElevatorHighGearMaxIntegralAccumulator = 500000; //todo: tune me
    public static final int kElevatorHighGearIZone = 0;
    public static final int kElevatorHighGearDeadband = 0;
    public static final int kElevatorHighGearCruiseVelocity = 12500;
    public static final int kElevatorHighGearAcceleration = 27000;//33000;
    public static final double kElevatorEpsilon = 1.0;//33000;
    public static final double kElevatorRampRate = 0.1;

    public static final double kWristKp = 3.0;
    public static final double kWristKi = 0.0;
    public static final double kWristKd = 50.0;
    public static final double kWristKf = 1.05;
    public static final double kWristJogKp = 2.0;
    public static final double kWristJogKd = 40.0;
    public static final double kWristKaWithCube = 0.006;
    public static final double kWristKaWithoutCube = 0.003;
    public static final double kWristKfMultiplierWithCube = 0.15;
    public static final double kWristKfMultiplierWithoutCube = 0.1;
    public static final double kWristElevatorAccelerationMultiplier = -1.0;
    public static final double kWristEpsilon = 2.0;

    public static final int kWristMaxIntegralAccumulator = 500000; //todo: tune me
    public static final int kWristIZone = 500; //todo: tune me
    public static final int kWristDeadband = 5; //todo: tune me
    public static final int kWristCruiseVelocity = 2500; //todo: tune me
    public static final int kWristAcceleration = 2500; //2000 //todo: tune me
    public static final double kWristRampRate = 0.001;
    public static final double kAutoWristRampRate = 0.01;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final double kLooperDt = 0.01;

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    // Elevator
    public static final int kElevatorMasterId = 10;
    public static final int kElevatorRightSlaveId = 11;
//    public static final int kElevatorLeftSlaveAId = 1;
//    public static final int kElevatorLeftSlaveBId = 2;

    // Wrist
    public static final int KWristMasterId = 15;

    // // Control Board
    // public static final boolean kUseGamepadForDriving = true;
    // public static final boolean kUseGamepadForButtons = false;

    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;

    // Height in in after applying turn factor.
    public static final double kElevatorLowSensitivityThreshold = 50.0;
    public static final double kLowSensitivityFactor = 1.0 / 4.0;

    public static final double kElevatorThrottleDeadband = 0.3;
    public static final double kMinShootTimeSec = 0.2;
    public static final double kJazzHandsEpsilon = 2.5;

    public static final double kKickstandToggleRumbleTime = 0.5;
    public static final double kKickstandDelay = 1.0;

    // // Pose of the LIDAR frame w.r.t. the robot frame
    // // TODO measure in CAD/on robot!
    // public static final double kLidarXOffset = -3.3211;
    // public static final double kLidarYOffset = 0.0;
    // public static final double kLidarYawAngleDegrees = 0.0;

    // /* LIDAR CONSTANTS */
    // public static final int kChezyLidarScanSize = 400;
    // public static final int kChezyLidarNumScansToStore = 10;
    // public static final String kChezyLidarPath = "/home/root/chezy_lidar";
    // public static final double kChezyLidarRestartTime = 2.5;

    // public static final String kLidarLogDir = "/home/lvuser/lidarLogs/";
    // public static final int kNumLidarLogsToKeep = 10;
    // public static final double kLidarICPTranslationEpsilon = 0.01; // convergence threshold for tx,ty
    // public static final double kLidarICPAngleEpsilon = 0.01;       // convergence threshold for theta

    public static final int kCameraStreamPort = 5810;

    /* LIDAR CONSTANTS */
    public static final double kScaleTrackerTimeout = 0.6;
}
