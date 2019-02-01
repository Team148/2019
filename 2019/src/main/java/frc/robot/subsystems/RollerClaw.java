/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class RollerClaw extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static RollerClaw m_instance;

  public RollerClaw() {

    super();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static RollerClaw getInstance() {
    if (m_instance == null) {
      m_instance = new RollerClaw();
    }
    return m_instance;
  }
}
