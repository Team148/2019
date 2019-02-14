/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FloorDiscIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static FloorDiscIntake m_instance;

  private final WPI_TalonSRX m_Disc1 = new WPI_TalonSRX(RobotMap.DISC_INTAKE);

  private final DoubleSolenoid m_DiscIntakeSolenoid = new DoubleSolenoid(RobotMap.PCM_ZERO, RobotMap.DISC_INTAKE_FORWARD, RobotMap.DISC_INTAKE_REVERSE);


  

  public FloorDiscIntake() {

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

  public static FloorDiscIntake getInstance() {
    if (m_instance == null) {
      m_instance = new FloorDiscIntake();
    }
    return m_instance;
  }

  public void setFactoryDefault(){
    m_Disc1.configFactoryDefault();
    m_Disc1.configFactoryDefault();

  }
  public void setBrakeMode(boolean mode){
    if (mode == true) {
      m_Disc1.setNeutralMode(NeutralMode.Brake);
      m_Disc1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_Disc1.setNeutralMode(NeutralMode.Coast);
      m_Disc1.setNeutralMode(NeutralMode.Coast);
    }
  }
  public void configureMotor(){
    m_Disc1.configOpenloopRamp(1.0, 0);
    m_Disc1.configOpenloopRamp(1.0, 0);
  }
  public void setMotorSafeties(){
    m_Disc1.setSafetyEnabled(false);
    m_Disc1.setSafetyEnabled(false);
  }

  public void setDiscIntakeMotor(double percent) {

    m_Disc1.set(ControlMode.PercentOutput, percent);
  }

  public double getAverageCurrent() {

    return m_Disc1.getOutputCurrent();
  }

  public void setDiscIntakeCylinder (boolean on) {
    if (on) {
      m_DiscIntakeSolenoid.set(Value.kForward);
    }
    else {
      m_DiscIntakeSolenoid.set(Value.kReverse);
    }
  }
}