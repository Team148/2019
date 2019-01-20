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

  //Map PDP CanID
  public static final int POWER_DISTRO_PANEL = 0;

  //Map LeftDrive CanIDs
  public static final int LEFT_DRIVE_MASTER = 1;
  public static final int LEFT_DRIVE_TWO = 2;
  public static final int LEFT_DRIVE_THREE = 3;

  //Map RightDrive CanIDs
  public static final int RIGHT_DRIVE_MASTER = 2;
  public static final int RIGHT_DRIVE_TWO = 4;
  public static final int RIGHT_DRIVE_THREE = 6;

  //Map Elevator CanIDs
  public static final int ELEVATOR_ONE = 7;
  public static final int ELEVATOR_TWO = 8;

  //Map Pigeon CanID
  public static final int PIGEON_IMU = 8;

  //Pneumatic Control Module
  public static final int PCM = 0;
}
