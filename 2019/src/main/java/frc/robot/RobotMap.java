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
  public static final int LEFT_DRIVE_MASTER = 10;
  public static final int LEFT_DRIVE_TWO = 9;
  public static final int LEFT_DRIVE_THREE = 8;

  public static final int RIGHT_DRIVE_MASTER = 1;
  public static final int RIGHT_DRIVE_TWO = 2;
  public static final int RIGHT_DRIVE_THREE = 3;

  //Map Elevator CAN IDs
  public static final int ELEVATOR_ONE = 4;
  public static final int ELEVATOR_TWO = 7;
  public static final int ELEVATOR_THREE = 11;

  //Map End Game CAN IDs
  public static final int ENDGAME_DRIVE = 12;
   
  //Map Roller Claw CAN IDs
  public static final int ROLLER_ONE = 5;

  //Map Ball Floor Intake CAN IDs
  public static final int BALL_INTAKE = 6;

  //Map Disc Floor Intake CAN IDs
  // public static final int DISC_INTAKE = 7;

  //Map Pigeon CAN ID
  public static final int PIGEON_IMU = 0;

  //Pneumatic Control Module
  public static final int PCM_ZERO  = 0;
  public static final int PCM_ONE = 1;

  //Solenoid Channels PCM = 0
  public static final int BALL_INTAKE_SOLENOID = 0;
  // public static final int WEDGE_SOLENOID = 1;
  public static final int FORKS_SOLENOID = 1;
  public static final int BEAK_4BAR_SOLENOID = 2;
  public static final int BEAK_GRAB_SOLENOID = 3;
  public static final int ENDGAME_SHIFT_FORWARD = 5;
  public static final int ENDGAME_SHIFT_REVERSE = 4;
  public static final int ENDGAME_FEET_FORWARD = 6;
  public static final int ENDGAME_FEET_REVERSE = 7;

  // // Solenoid Channels PCM = 1

  //Mexico Sensor
  public static final int MEXICO_SENSOR_ONE = 0;

}
