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

  //Map LeftDrive CAN IDs
  public static final int LEFT_DRIVE_MASTER = 1;
  public static final int LEFT_DRIVE_TWO = 3;
  public static final int LEFT_DRIVE_THREE = 5;

  //Map RightDrive CAN IDs
  public static final int RIGHT_DRIVE_MASTER = 2;
  public static final int RIGHT_DRIVE_TWO = 4;
  public static final int RIGHT_DRIVE_THREE = 6;

  //Map Elevator CanIDs
  public static final int ELEVATOR_ONE = 10;
  public static final int ELEVATOR_TWO = 11;

  //Map Ball Floor Intake CAN IDs
  public static final int BALL_FLOOR_ONE = 9;


  //Map Pigeon CAN ID
  public static final int PIGEON_IMU = 0;

  //Pneumatic Control Module
  public static final int PCM = 0;
}
