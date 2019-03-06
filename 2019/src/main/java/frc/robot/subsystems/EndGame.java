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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class EndGame extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static EndGame m_instance;

  private boolean m_EndGameShifted = false;
  private boolean m_AnklesReleased = false;


  private final DoubleSolenoid m_endGameShifter = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_SHIFT_FORWARD, RobotMap.ENDGAME_SHIFT_REVERSE);
  private final DoubleSolenoid m_endGameAnkles = new DoubleSolenoid(RobotMap.PCM_ONE, RobotMap.ENDGAME_FEET_FORWARD, RobotMap.ENDGAME_FEET_REVERSE);

  private final WPI_TalonSRX m_EndGameDrive = new WPI_TalonSRX(RobotMap.ENDGAME_DRIVE);

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

  private void setFactoryDefault() {
    m_EndGameDrive.configFactoryDefault();
  }

  private void configureMotors() {

    m_EndGameDrive.set(ControlMode.PercentOutput, 0.0);

    m_EndGameDrive.configOpenloopRamp(1.0, 0);

    m_EndGameDrive.configVoltageCompSaturation(12.0, 0);
    m_EndGameDrive.enableVoltageCompensation(true);

    m_EndGameDrive.configNominalOutputForward(0.0, 0);
    m_EndGameDrive.configNominalOutputReverse(0.0, 0);
    
  }

  public void setEndGameShifted (boolean on) {


    if (on) {
      m_endGameShifter.set(Value.kForward);
      m_EndGameShifted = true;
    }
    else {
      m_endGameShifter.set(Value.kReverse);
    }
  }

  public void setDriveForward(boolean on){
    if(on)
      m_EndGameDrive.set(ControlMode.PercentOutput, -Constants.ENDGAME_DRIVE_SPEED);
    else
      m_EndGameDrive.set(ControlMode.PercentOutput, 0);
  }

  public void setAnklesReleased (boolean on) {

    if (on) {
      m_endGameAnkles.set(Value.kForward);
      m_AnklesReleased = true;
    }
    else {
      m_endGameAnkles.set(Value.kReverse);
    }
  }

  public boolean getEndGameShifted(){
    return m_EndGameShifted;
  }

  public boolean getAnklesReleased(){
    return m_AnklesReleased;
  }

}
