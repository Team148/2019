package frc.robot;

public class Constants {

    //DTs
    public static final double AUTO_PERIODIC_DT = 0.010;
    public static final double TELE_PERIODIC_DT = 0.020;
    public static final double kLooperDt = 0.01;
    
    //Drivetrain Constants
    public static final int DRIVE_ENCODER_TPR = 5000;

    public static final double HEADING_P = 0.7;
    public static final double DRIVE_FEEDFORWARD = 0.05;

    //Turn to LimeLight Constants
    public static final double LL_K_P = 0.04;
    public static final double LL_K_I = 0;
    public static final double LL_K_D = 0.0;//LL_K_P * 0.25;

    public static final double kCameraClose = 10.0;
    public static final double kCameraMid = 15.0;
    public static final double kCameraFar = 20.0;
    public static final double kCameraDriveClose = 0.06;
    public static final double kCameraDriveMid = 0.035;
    public static final double kCameraDriveFar = 0.02;


    //Elevator Constants

    public static final int ELEVATOR_SOFT_LIMIT = 52650;

    public static final double ELEVATOR_UP_OUTPUT_PERCENT = 1.0;
    public static final double ELEVATOR_DOWN_OUTPUT_PERCENT = -0.4;

    public static final double ELEVATOR_F_UP = 0.08;
    public static final double ELEVATOR_F_DOWN = ELEVATOR_F_UP - 0.030;
    // public static final double ELEVATOR_ZERO_F = -0.0732;
    public static final double ELEVATOR_ZERO_F = -0.1;
    // public static final double ELEVATOR_ZERO_F = 0.0;

    public static final double ELEVATOR_P = 0.04;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.4;

    public static final double ELEVATOR_F_VELOCITY = 0.26;
    public static final double ELEVATOR_P_VELOCITY = 0.7;
    public static final double ELEVATOR_I_VELOCITY = 0.0;
    public static final double ELEVATOR_D_VELOCITY = 4.0;
    public static final double ELEVATOR_AUX_F_VELOCITY = 0.1367;

    public static final double ELEVATOR_ZERO = 0.0;

    public static final double ELEVATOR_LOW_GOAL = 2700;
    public static final double ELEVATOR_CARGO = 11500;
    public static final double ELEVATOR_MIDDLE = 22950;
    public static final double ELEVATOR_HIGH = 47250;
    public static final double ELEVATOR_MAX_HEIGHT = 52650;

    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION = 3000.0;
    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND = 1000.0;

    public static final double ELEVATOR_ERROR_TOLERANCE = 2000.0;
    public static final double ELEVATOR_MANUAL_DEADBAND_UP = 0.2;
    public static final double ELEVATOR_MANUAL_DEADBAND_DOWN = -0.2;
    public static final double ELEVATOR_MANUAL_DPOS_SCALAR = 10000.0;


    //ENDGAME Constants

    public static final double ENDGAME_F_UP = -0.10;
    public static final double ENDGAME_P = 0.05;
    public static final double ENDGAME_I = 0.0;
    public static final double ENDGAME_D = 0.50;

    public static final double ENDGAME_UP_OUTPUT_PERCENT = 0.6;
    public static final double ENDGAME_DOWN_OUTPUT_PERCENT = -0.6;

    public static final int ENDGAME_ZERO = -4350; //if feet are slightly out since they can't go all the way back in
    public static final int ENDGAME_ZERO_ANKLES = -33500; //if ankles are deployed, zero is even lower to ensure we don't damage the robot

    public static final int ENDGAME_TOP = -80000;
    // public static final int ENDGAME_TOP = -74500;
    public static final int ENDGAME_CHILL = -66700;
    public static final int ENDGAME_CHILL2 = -55000; //60000

    public static final int ENDGAME_FINISH_TOLERANCE  = 5000; //guess

    public static final double ENDGAME_DRIVE_SPEED = 0.25;


    //Output Percents
    public static final double ELEVATOR_OUTPUT_PERCENT = 1.0;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 18.9;
    // public static final double kDriveWheelDiameterInches = 3.8;// * 0.99;
    // public static final double kDriveWheelDiameterInches = 4.08524;
    public static final double kDriveWheelDiameterInches = 3.94;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  //Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 44.9;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.125;  // V per rad/s
    public static final double kDriveKa = 0.01;  // V per rad/s^2

    // Geometry
    // public static final double kCenterToFrontBumperDistance = 14.99;	//34.473 / 2.0;
    // public static final double kCenterToRearBumperDistance = 19.48;		//34.473 / 2.0;
    // public static final double kCenterToSideBumperDistance = 32.47 / 2.0;
    public static final double kCenterToFrontBumperDistance = 18.64;	//34.473 / 2.0;
    public static final double kCenterToRearBumperDistance = 18.64;		//34.473 / 2.0;
    public static final double kCenterToSideBumperDistance = 28.12 / 2.0;

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
    // public static final double kDriveLowGearVelocityKp = 0.0;
    public static final double kDriveLowGearVelocityKp = 0.7;	//0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    // public static final double kDriveLowGearVelocityKd = 0.0;
    public static final double kDriveLowGearVelocityKd = 7.7;	//10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    //PID Drive Position Gains
    public static final double kDriveLowGearPositionKp = 0.4;//0.02;//0.025;//0.0002;// 1.0/5000;	//0.9;
    public static final double kDriveLowGearPositionKi = 0.002;//0.0001;// 0.002/5000;
    public static final double kDriveLowGearPositionKd = 60.0;//2.0;//0.2;//0.25;//0.02; //100.0/5000;	//10.0;
    public static final double kDriveLowGearPositionKf = 0.0;//0.32;
    public static final int kDriveLowGearPositionIZone = 700;
    public static final double kDriveLowGearPositionRampRate = 240.0; // V/s
    public static final double kDriveLowGearNominalOutput = 0.5; // V
    public static final double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
                                                                                                               // in RPM
    public static final double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
           
    //Aiming gains
    public static final double kP_aim = 0.002;

    //CAN timeouts
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
}
