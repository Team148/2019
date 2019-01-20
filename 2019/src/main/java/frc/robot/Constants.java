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
    public static final double ELEVATOR_MAX_HEIGHT = 34000;

    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION = 3000.0;
    public static final double ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND = 1000.0;

    public static final double ELEVATOR_ERROR_TOLERANCE = 2000.0;
    public static final double ELEVATOR_MANUAL_DEADBAND = 0.2;
    public static final double ELEVATOR_MANUAL_DPOS_SCALAR = 12000.0;

    //Output Percents
    public static final double ELEVATOR_OUTPUT_PERCENT = 1.0;
}
