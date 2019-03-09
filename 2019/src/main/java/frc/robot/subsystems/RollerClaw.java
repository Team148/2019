/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class RollerClaw extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static RollerClaw m_instance;

  private final WPI_TalonSRX m_Roller1 = new WPI_TalonSRX(RobotMap.ROLLER_ONE);
  

  public RollerClaw() {

    super();

    setFactoryDefault();
    setBrakeMode(true);
    setMotorSafeties();


  }
  public static RollerClaw getInstance(){
    if (m_instance == null) {
      m_instance = new RollerClaw();
    }
    return m_instance;
  }

  private void setFactoryDefault() {
    m_Roller1.configFactoryDefault();
  }

  private void setBrakeMode(boolean mode) {
    if (mode == true) {
      m_Roller1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_Roller1.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setMotorSafeties() {
    m_Roller1.setSafetyEnabled(false);
  }

  // private void configureMotor() {
  //   m_Roller1.configOpenloopRamp(1.0, 0);
  // }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setRollerClaw (double percent) {
    m_Roller1.set(percent);
  }
}
