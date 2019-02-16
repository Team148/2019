/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Beak extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Beak m_instance;

  private final Solenoid m_beakBar = new Solenoid(RobotMap.PCM_ZERO, RobotMap.BEAK_4BAR_SOLENOID);
  private final Solenoid m_beakGrab = new Solenoid(RobotMap.PCM_ZERO, RobotMap.BEAK_GRAB_SOLENOID);

  public Beak() {

    super();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static Beak getInstance() { 
    if (m_instance == null) {
      m_instance = new Beak();
    }
    return m_instance;
  }

  public void setBeakBar (boolean on) {
    if (on) {
      m_beakBar.set(true);
    }
    else {
      m_beakBar.set(false);
    }
  }

  public void setBeakGrab (boolean on) {
    if (on) {
      m_beakGrab.set(true);
    }
    else {
      m_beakGrab.set(false);
    }
  }
}
