/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //Map PDP CAN ID
  public static final int POWER_DISTRO_PANEL = 0;

  // //Map Drive CAN IDs
  public static final int LEFT_DRIVE_MASTER = 1;
  public static final int LEFT_DRIVE_TWO = 2;
  public static final int LEFT_DRIVE_THREE = 3;

  public static final int RIGHT_DRIVE_MASTER = 10;
  public static final int RIGHT_DRIVE_TWO = 9;
  public static final int RIGHT_DRIVE_THREE = 8;

  //Map Elevator CAN IDs
  public static final int ELEVATOR_MASTER = 4;
  public static final int ELEVATOR_FOLLOW = 7;
   
  //Map Roller Claw CAN IDs
  public static final int ROLLER_ONE = 5;

  //Map Ball Floor Intake CAN IDs
  public static final int BALL_INTAKE = 6;

  //Map EndGame CAN IDs
  public static final int ENDGAME_DRIVE = 11;

  //Map Pigeon CAN ID
  public static final int PIGEON_IMU = 0;

  //Pneumatic Control Module
  public static final int PCM_ZERO  = 0;
  public static final int PCM_ONE = 1;

  //Solenoid Channels PCM = 0
  public static final int BALL_INTAKE_SOLENOID = 0;
  public static final int DISC_INTAKE_SOLENOID = 1;
  public static final int BEAK_4BAR_SOLENOID = 2;
  public static final int BEAK_GRAB_SOLENOID = 3;
  public static final int ENDGAME_SHIFT_FORWARD = 4;
  public static final int ENDGAME_SHIFT_REVERSE = 5;
  public static final int ENDGAME_FOOT_FORWARD = 6;
  public static final int ENDGAME_FOOT_REVERSE = 7;

  // Solenoid Channels PCM = 1

}
