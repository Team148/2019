/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Elevator m_instance;

  private boolean m_isClosedLoop = false;
  private double m_position = 1.0;

  //Declare Elevator TalonSRXs
  private final WPI_TalonSRX m_elevator1 = new WPI_TalonSRX(RobotMap.ELEVATOR_ONE);

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
    m_elevator1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_elevator1.setSensorPhase(true);
    m_elevator1.setSelectedSensorPosition(0, 0, 0);
    
  }

  public void joystickControl(double yStick) {
    m_elevator1.set(ControlMode.PercentOutput, yStick * Constants.ELEVATOR_OUTPUT_PERCENT);
  }

  public void configOpenLoop() {

  }

  public void configClosedLoop() {

    m_elevator1.configVoltageCompSaturation(12.0, 0);
    m_elevator1.enableVoltageCompensation(true);

    m_elevator1.configNominalOutputForward(0.0, 0);
    m_elevator1.configNominalOutputReverse(0.0, 0);

    m_elevator1.configPeakOutputForward(Constants.ELEVATOR_UP_OUTPUT_PERCENT, 0);
    m_elevator1.configPeakOutputReverse(Constants.ELEVATOR_DOWN_OUTPUT_PERCENT, 0);

    m_elevator1.configForwardSoftLimitThreshold(Constants.ELEVATOR_SOFT_LIMIT);

    m_elevator1.set(ControlMode.Position, 0.0);
    m_elevator1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // m_elevator1.setSensorPhase(false);

    m_elevator1.configClosedloopRamp(0.10, 0);

    m_elevator1.config_kF(0, 0, 0);
    m_elevator1.config_kP(0, Constants.ELEVATOR_P, 0);
    m_elevator1.config_kI(0, Constants.ELEVATOR_I, 0);
    m_elevator1.config_kD(0, Constants.ELEVATOR_D, 0);

    m_elevator1.setSelectedSensorPosition(0, 0, 0);

    m_isClosedLoop = true;
  }

  public void configClosedLoopMagic(int cruiseVelocity, int acceleration) {

    m_elevator1.set(ControlMode.MotionMagic, 0.0);
    m_elevator1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // m_elevator1.setSensorPhase(false);

    m_elevator1.configNominalOutputForward(0.0, 0);
    m_elevator1.configNominalOutputReverse(0.0, 0);

    m_elevator1.configPeakOutputForward(1.0, 0);
    m_elevator1.configPeakOutputReverse(-1.0, 0);

    m_elevator1.configVoltageCompSaturation(12.0, 0);
    m_elevator1.enableVoltageCompensation(true);

    m_elevator1.configClosedloopRamp(0.0, 0);

    m_elevator1.config_kF(0, Constants.ELEVATOR_F_VELOCITY, 0);
    m_elevator1.config_kP(0, Constants.ELEVATOR_P_VELOCITY, 0);
    m_elevator1.config_kI(0, Constants.ELEVATOR_I_VELOCITY, 0);
    m_elevator1.config_kD(0, Constants.ELEVATOR_D_VELOCITY, 0);

    m_elevator1.configVelocityMeasurementWindow(32, 0);
    m_elevator1.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);

    m_elevator1.configMotionCruiseVelocity(cruiseVelocity, 0);
    m_elevator1.configMotionAcceleration(acceleration, 0);

    m_elevator1.setNeutralMode(NeutralMode.Brake);
  }

  public void configNeutralClosedLoop() {

    m_elevator1.config_kP(0, Constants.ELEVATOR_P, 0);
    m_elevator1.config_kI(0, Constants.ELEVATOR_I, 0);
    m_elevator1.config_kD(0, Constants.ELEVATOR_D, 0);
  }

  public boolean isClosedLoop() {

    return m_isClosedLoop;
  }

  public int getElevatorPosition() {

    return m_elevator1.getSelectedSensorPosition(0);
  }

  public int getElevatorVelocity() {

    return m_elevator1.getSelectedSensorVelocity(0);
  }

  public void setElevatorPosition(double position, double feedforward) {

    if (!m_isClosedLoop) {
      configClosedLoop();
    }

    m_position = position;

    if (m_position < 1) {
      m_position = 0;
    }

    m_elevator1.set(ControlMode.Position, m_position, DemandType.ArbitraryFeedForward, feedforward);
  }

  public void setElevatorPositionMagic(double position, double feedforward) {

    m_position = position;

    if (m_position < 1) {
      m_position = 0;
    }

    m_elevator1.set(ControlMode.MotionMagic, m_position, DemandType.ArbitraryFeedForward, feedforward);
  }

  public void setElevatorEncoderZero() {

    m_elevator1.setSelectedSensorPosition(0, 0, 0);
  }

  public void incrementElevatorPosition(double position) {

    if (!m_isClosedLoop) {
      configClosedLoop();
    }

    double localPosition = m_position;
    localPosition += position;

    if (getElevatorPosition() > Constants.ELEVATOR_MAX_HEIGHT && position > 0) {
      localPosition = getElevatorPosition();
    }

    setElevatorPosition(localPosition, Constants.ELEVATOR_F_UP);
  }
}
