/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  private final WPI_TalonSRX m_endGame1 = new WPI_TalonSRX(RobotMap.ENDGAME_DRIVE);

  private final DoubleSolenoid m_endGameShift = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_SHIFT_FORWARD, RobotMap.ENDGAME_SHIFT_REVERSE);
  private final DoubleSolenoid m_endGameFoot = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_FOOT_FORWARD, RobotMap.ENDGAME_FOOT_REVERSE);

  public EndGame() {

    super();

    setFactoryDefault();
    setBrakeMode(true);
    setMotorSafeties();
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

  private void setFactoryDefault() {
    m_endGame1.configFactoryDefault();
  }

  private void setBrakeMode(boolean mode) {
    if (mode == true) {
      m_endGame1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_endGame1.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setMotorSafeties() {
    m_endGame1.setSafetyEnabled(false);
  }

  public void setEndGameShift (boolean on) {
    if (on) {
      m_endGameShift.set(Value.kForward);
    }
    else {
      m_endGameShift.set(Value.kReverse);
    }
  }

  public void setEndGameLatch (boolean on) {
    if (on) {
      m_endGameFoot.set(Value.kForward);
    }
    else {
      m_endGameFoot.set(Value.kReverse);
    }
  }

}
