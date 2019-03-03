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
public class EndGame extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static EndGame m_instance;

  private final DoubleSolenoid m_endGameShock = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_SHIFT_FORWARD, RobotMap.ENDGAME_SHIFT_REVERSE);
  private final DoubleSolenoid m_endGameLatch = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_FEET_FORWARD, RobotMap.ENDGAME_FEET_REVERSE);

  public EndGame() {

    super();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static EndGame getInstance() {
    if (m_instance == null) {
      m_instance = new EndGame();
    }
    return m_instance;
  }

  public void setEndGameShocks (boolean on) {
    if (on) {
      m_endGameShock.set(Value.kForward);
    }
    else {
      m_endGameShock.set(Value.kReverse);
    }
  }

  public void setEndGameLatch (boolean on) {
    if (on) {
      m_endGameLatch.set(Value.kForward);
    }
    else {
      m_endGameLatch.set(Value.kReverse);
    }
  }

}
