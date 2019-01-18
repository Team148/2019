/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Drivetrain m_instance;

  //Create Left Drive TalonSRXs
  private final WPI_TalonSRX m_driveLeft1= new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MASTER);
  private final WPI_TalonSRX m_driveLeft2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_TWO);
  private final WPI_TalonSRX m_driveLeft3 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_THREE);

  //Create Right Drive TalonSRXs
  private final WPI_TalonSRX m_driveRight1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MASTER);
  private final WPI_TalonSRX m_driveRight2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_TWO);
  private final WPI_TalonSRX m_driveRight3 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_THREE);

  private final PigeonIMU m_driveGyro = new PigeonIMU(RobotMap.PIGEON_IMU);
  private double[] yawPitchRoll = new double[3];

  private DifferentialDrive m_drive;

  public Drivetrain() {

    super();

    setFactoryDefault();
    setBrakeMode(false);
    configureMotors();
    setMotorSafeties();
    setPigeonStatusFrame();
    
    m_drive = new DifferentialDrive(m_driveLeft1, m_driveRight1);
    m_drive.setSafetyEnabled(false);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoystick());
  }

  public static Drivetrain getInstance() {
    if (m_instance == null) {
      m_instance = new Drivetrain();
    }
    return m_instance;
  }

  private void configureMotors() {
    m_driveLeft2.follow(m_driveLeft1);
    m_driveLeft3.follow(m_driveLeft1);

    m_driveRight2.follow(m_driveRight1);
    m_driveRight3.follow(m_driveRight1);

    m_driveLeft1.setInverted(false);
    m_driveLeft2.setInverted(false);
    m_driveLeft3.setInverted(false);

    m_driveRight1.setInverted(true);
    m_driveRight2.setInverted(true);
    m_driveRight3.setInverted(true);

    m_driveLeft1.configNominalOutputForward(0.0, 0);
    m_driveRight2.configNominalOutputForward(0.0, 0);

    m_driveLeft1.configPeakOutputForward(1.0, 0);
    m_driveLeft1.configPeakOutputReverse(-1.0, 0);

    m_driveRight1.configPeakOutputForward(1.0, 0);
    m_driveRight1.configPeakOutputReverse(-1.0, 0);
  }

  private void setFactoryDefault() {
    m_driveLeft1.configFactoryDefault();
    m_driveLeft2.configFactoryDefault();
    m_driveLeft3.configFactoryDefault();
    m_driveRight1.configFactoryDefault();
    m_driveRight2.configFactoryDefault();
    m_driveRight3.configFactoryDefault();
  }
  private void setBrakeMode(boolean mode) {

    if (mode == true) {
      m_driveLeft1.setNeutralMode(NeutralMode.Brake);
      m_driveLeft2.setNeutralMode(NeutralMode.Brake);
      m_driveLeft3.setNeutralMode(NeutralMode.Brake);
      m_driveRight1.setNeutralMode(NeutralMode.Brake);
      m_driveRight2.setNeutralMode(NeutralMode.Brake);
      m_driveRight3.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_driveLeft1.setNeutralMode(NeutralMode.Coast);
      m_driveLeft2.setNeutralMode(NeutralMode.Coast);
      m_driveLeft3.setNeutralMode(NeutralMode.Coast);
      m_driveRight1.setNeutralMode(NeutralMode.Coast);
      m_driveRight2.setNeutralMode(NeutralMode.Coast);
      m_driveRight3.setNeutralMode(NeutralMode.Coast);
    }
  }

  private void setMotorSafeties() {
    m_driveLeft1.setSafetyEnabled(false);
    m_driveLeft2.setSafetyEnabled(false);
    m_driveLeft3.setSafetyEnabled(false);
    m_driveRight1.setSafetyEnabled(false);
    m_driveRight2.setSafetyEnabled(false);
    m_driveRight3.setSafetyEnabled(false);
  }

  private void setPigeonStatusFrame() {
    m_driveGyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 5, 0);
  }

  private double getGyroYaw() {
    m_driveGyro.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0];
  }

  private double getGyroPitch() {
    m_driveGyro.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[1];
  }

  private double getGyroRoll() {
    m_driveGyro.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[2];
  }

  private double updatePigeon() {
    m_driveGyro.getRawGyro(yawPitchRoll);
    return yawPitchRoll[0];
  }

  public void getPigeonStatus() {
    PigeonIMU.GeneralStatus generalStatus = new PigeonIMU.GeneralStatus();
    m_driveGyro.getGeneralStatus(generalStatus);
  }

  public void arcadeMode(double xstick, double ystick) {
    m_drive.arcadeDrive(xstick, ystick);
  }

  public void tankMode(double leftStick, double rightStick) {
    m_drive.tankDrive(leftStick, rightStick);
  }

  public void setLeftRight(double left, double right) {
    m_driveLeft1.set(left);
    m_driveRight1.set(right);
  }
}