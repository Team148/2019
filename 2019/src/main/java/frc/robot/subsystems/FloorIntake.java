/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FloorIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static FloorIntake m_instance;

  private final WPI_TalonSRX m_Floor1 = new WPI_TalonSRX(RobotMap.FLOOR_INTAKE);

  public FloorIntake() {

    super();

    setFactoryDefault();
    setBrakeMode(true);
    configureMotor();
    setMotorSafeties();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static FloorIntake getInstance() {
    if (m_instance == null) {
      m_instance = new FloorIntake();
    }
    return m_instance;
  }

  private void setFactoryDefault() {

    m_Floor1.configFactoryDefault();
  }

  private void setBrakeMode(boolean mode) {

    if (mode == true) {
      m_Floor1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_Floor1.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void configureMotor() {

    m_Floor1.configVoltageCompSaturation(12.0, 0);
    m_Floor1.enableVoltageCompensation(true);
  }

  private void setMotorSafeties() {
    
    m_Floor1.setSafetyEnabled(false);
  }

  public void setFloorIntakeMotor(double percent) {

    m_Floor1.set(ControlMode.PercentOutput, percent);
  }

  public double getAverageCurrent() {

    return m_Floor1.getOutputCurrent();
  }
}
