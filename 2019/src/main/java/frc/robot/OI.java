package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI m_instance = null;

  //Constants for the Joystick ports
  public static final int DRIVER_JOYSTICK_PORT = 0;
  public static final int OPERATOR_JOYSTICK_PORT = 1;
  public static final int DRIVER_STATION_SWITCHES = 2;

  //Create the Joysticks for both Driver and Operator
  public final Joystick m_driveJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
  public final Joystick m_operatorJoystick = new Joystick(OPERATOR_JOYSTICK_PORT);
  public final Joystick m_driveStation = new Joystick(DRIVER_STATION_SWITCHES);

  public OI() {

    //Create Driver Joystick Buttons and POVs
    final JoystickButton m_driverA = new JoystickButton(m_driveJoystick, 1);
    final JoystickButton m_driverB = new JoystickButton(m_driveJoystick, 2);
    final JoystickButton m_driverX = new JoystickButton(m_driveJoystick, 3);
    final JoystickButton m_driverY = new JoystickButton(m_driveJoystick, 4);
    final JoystickButton m_driverLB = new JoystickButton(m_driveJoystick, 5);
    final JoystickButton m_driverRB = new JoystickButton(m_driveJoystick, 6);
    final JoystickButton m_driverSelect = new JoystickButton(m_driveJoystick, 7);
    final JoystickButton m_driverStart = new JoystickButton(m_driveJoystick, 8);
    final JoystickButton m_driverLeftStick = new JoystickButton(m_driveJoystick, 9);
    final JoystickButton m_driverRightStick = new JoystickButton(m_driveJoystick, 10);
    final POVButton m_driverPOVUp = new POVButton(m_driveJoystick, 90);
    final POVButton m_driverPOVDown = new POVButton(m_driveJoystick, 270);
    final POVButton m_driverPOVLeft = new POVButton(m_driveJoystick, 180);
    final POVButton m_driverPOVRight = new POVButton(m_driveJoystick, 0);

    //Create Operator Joystick Buttons and POVs
    final JoystickButton m_operatorA = new JoystickButton(m_operatorJoystick, 1);
    final JoystickButton m_operatorB = new JoystickButton(m_operatorJoystick, 2);
    final JoystickButton m_operatorX = new JoystickButton(m_operatorJoystick, 3);
    final JoystickButton m_operatorY = new JoystickButton(m_operatorJoystick, 4);
    final JoystickButton m_operatorLB = new JoystickButton(m_operatorJoystick, 5);
    final JoystickButton m_operatorRB = new JoystickButton(m_operatorJoystick, 6);
    final JoystickButton m_operatorSelect = new JoystickButton(m_operatorJoystick, 7);
    final JoystickButton m_operatorStart = new JoystickButton(m_operatorJoystick, 8);
    final JoystickButton m_operatorLeftStick = new JoystickButton(m_operatorJoystick, 9);
    final JoystickButton m_operatorRightStick = new JoystickButton(m_operatorJoystick, 10);
    final POVButton m_operatorPOVUp = new POVButton(m_operatorJoystick, 90);
    final POVButton m_operatorPOVDown = new POVButton(m_operatorJoystick, 270);
    final POVButton m_operatorPOVLeft = new POVButton(m_operatorJoystick, 180);
    final POVButton m_operatorPOVRight = new POVButton(m_operatorJoystick, 0);

    // //Create Driver Station switches
    final JoystickButton m_dsSwitch1 = new JoystickButton(m_driveStation, 12);
    final JoystickButton m_dsSwitch2 = new JoystickButton(m_driveStation, 13);
    final JoystickButton m_dsSwitch3 = new JoystickButton(m_driveStation, 14);
    final JoystickButton m_dsSwitch4 = new JoystickButton(m_driveStation, 15);
    final JoystickButton m_dsSwitch5 = new JoystickButton(m_driveStation, 16);
      }

  public static OI getInstance() {
    if(m_instance == null) {
      m_instance = new OI();
    }
    return m_instance;
  }

  //Get DS Switches
  public boolean getBallMode() {
    return m_driveStation.getRawButton(12);
  }
  public boolean getDiscGrabWithSensor() {
    return m_driveStation.getRawButton(13);
  }
  public boolean getEndgameManual() {
    return m_driveStation.getRawButton(15);
  }
  public boolean getEndgameSafety() {
    return m_driveStation.getRawButton(16);
  }

  //Get Driver
  public double getThrottle() {
    return m_driveJoystick.getRawAxis(1);
  }
  public double getTurn() {
    return m_driveJoystick.getRawAxis(4);
  }
  public boolean getDriverQuarterSpeed() {
    return m_driveJoystick.getRawButton(1);
  }
  public boolean getDriverHalfSpeed() {
    return m_driveJoystick.getRawButton(3);
  }
  public boolean getDriverThreeQuarterSpeed() {
    return m_driveJoystick.getRawButton(2);
  }
  public boolean getDriver4() {
    return m_driveJoystick.getRawButton(4);
  }
  public boolean getDriver5() {
    return m_driveJoystick.getRawButton(5);
  }
  public boolean getDriver6() {
    return m_driveJoystick.getRawButton(6);
  }
  public boolean getDriverEndgame() {
    return (m_driveJoystick.getRawButton(7) && m_driveJoystick.getRawButton(8));
  }
  public boolean getDriverVision(){
    return m_driveJoystick.getRawButton(10);
  }

  //Get Operator
  public double getElevatorManual() {
    return m_operatorJoystick.getRawAxis(1);
  }
  public double getDeployIntakes() {
    return m_operatorJoystick.getRawAxis(2);
  }
  public double getRetractIntakes() {
    return m_operatorJoystick.getRawAxis(3);
  }
  public boolean getOperator1() {
    // return m_operatorJoystick.getRawButton(1);
    return m_operatorJoystick.getRawButtonPressed(1);
  }
  public boolean getOperator2() {
    return m_operatorJoystick.getRawButton(2);
  }
  public boolean getOperator3() {
    return m_operatorJoystick.getRawButton(3);
  }
  public boolean getOperator4() {
    return m_operatorJoystick.getRawButton(4);
  }
  // public boolean getOperator4BarIn() {
  //   return m_operatorJoystick.getRawButton(1);
  // }
  // public boolean getOperatorHandoff() {
  //   return m_operatorJoystick.getRawButton(2);
  // }
  // public boolean getOperator4BarOut() {
  //   return m_operatorJoystick.getRawButton(3);
  // }
  // public boolean getOperatorDiscIntakeUp() {
  //   return m_operatorJoystick.getRawButton(4);
  // }
  public boolean getFloorIntake() {
    return m_operatorJoystick.getRawButton(5);
  }
  public boolean getFloorOuttake() {
    return m_operatorJoystick.getRawButton(6);
  }
  public boolean getOperatorEndgame() {
    return (m_operatorJoystick.getRawButton(7) && m_operatorJoystick.getRawButton(8));
  }
}
