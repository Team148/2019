/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Beak extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Beak m_instance;

  private final DoubleSolenoid m_beakIn = new DoubleSolenoid(RobotMap.PCM_ZERO, RobotMap.BEAK_IN_FORWARD, RobotMap.BEAK_IN_REVERSE);
  private final DoubleSolenoid m_beakGrab = new DoubleSolenoid(RobotMap.PCM_ZERO, RobotMap.BEAK_GRAB_FORWARD, RobotMap.BEAK_GRAB_REVERSE);

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

  public void setBeakIn (boolean on) {
    if (on) {
      m_beakIn.set(Value.kForward);
    }
    else {
      m_beakIn.set(Value.kReverse);
    }
  }

  public void setBeakGrab (boolean on) {
    if (on) {
      m_beakGrab.set(Value.kForward);
    }
    else {
      m_beakGrab.set(Value.kReverse);
    }
  }
}
