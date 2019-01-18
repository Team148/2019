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
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Elevator m_instance;

  //Declare Elevator TalonSRXs
  private final WPI_TalonSRX m_elevator1 = new WPI_TalonSRX(8);

  public Elevator() {

    super();

    setFactoryDefault();
    setBrakeMode(true);
    configureMotors();
    setMotorSafeties();

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }

  private void setFactoryDefault() {
    m_elevator1.configFactoryDefault();
  }

  private void setBrakeMode(boolean mode) {
    if (mode == true) {
      m_elevator1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_elevator1.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void setMotorSafeties() {
    m_elevator1.setSafetyEnabled(false);
  }

  private void configureMotors() {
    m_elevator1.configOpenloopRamp(1.0, 0);
  }

  public void joystickControl(double yStick) {
    m_elevator1.set(ControlMode.PercentOutput, yStick * Constants.ELEVATOR_OUTPUT_PERCENT);
  }
}
