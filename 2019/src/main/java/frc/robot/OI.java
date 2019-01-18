/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

  //Create the Joysticks for both Driver and Operator
  public final Joystick m_driveJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
  public final Joystick m_operatorJoystick = new Joystick(OPERATOR_JOYSTICK_PORT);

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

    //Buttons to Commands
    
  }

  public static OI getInstance() {
    if(m_instance == null) {
      m_instance = new OI();
    }
    return m_instance;
  }
}