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

    //DTs
    public static final double AUTO_PERIODIC_DT = 0.010;
    public static final double TELE_PERIODIC_DT = 0.020;
    public static final double kLooperDt = 0.01;
    
    //Drivetrain Constants
    public static final int DRIVE_ENCODER_TPR = 5000;

    //Elevator Constants
    public static final int ELEVATOR_SOFT_LIMIT = 39000;
    public static final double ELEVATOR_UP_OUTPUT_PERCENT = 1.0;
    public static final double ELEVATOR_DOWN_OUTPUT_PERCENT = -0.4;

    public static final double ELEVATOR_F_UP = 0.1367;
    public static final double ELEVATOR_F_DOWN = ELEVATOR_F_UP - 0.025;
    public static final double ELEVATOR_ZERO_F = -0.0732;
    // public static final double ELEVATOR_ZERO_F = 0.0;
    public static final double ELEVATOR_P = 0.05;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.50;

    public static final double ELEVATOR_F_VELOCITY = 0.26;
    public static final double ELEVATOR_P_VELOCITY = 0.7;
    public static final double ELEVATOR_I_VELOCITY = 0.0;
    public static final double ELEVATOR_D_VELOCITY = 4.0;
    public static final double ELEVATOR_AUX_F_VELOCITY = 0.1367;

    public static final double ELEVATOR_ZERO = 0.0;
    public static final double ELEVATOR_CARGO = 8000;
    public static final double ELEVATOR_MIDDLE = 17000;
    public static final double ELEVATOR_HIGH = 35000;
    public static final double ELEVATOR_MAX_HEIGHT = 39000;

    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION = 3000.0;
    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND = 1000.0;

    public static final double ELEVATOR_ERROR_TOLERANCE = 2000.0;
    public static final double ELEVATOR_MANUAL_DEADBAND_UP = 0.2;
    public static final double ELEVATOR_MANUAL_DEADBAND_DOWN = -0.2;
    public static final double ELEVATOR_MANUAL_DPOS_SCALAR = 10000.0;

    //Output Percents
    public static final double ELEVATOR_OUTPUT_PERCENT = 1.0;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 18.9;
    public static final double kDriveWheelDiameterInches = 3.8;// * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.2;  //Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 44.9;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

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
    public static final double kDriveLowGearVelocityKp = 0.7;	//0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 7.7;	//10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    //CAN timeouts
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
}
