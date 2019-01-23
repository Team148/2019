/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.RobotState;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.DriveWithJoystick;
import frc.planners.DriveMotionPlanner;

import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;

import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;

import lib.util.DriveSignal;
import lib.util.ReflectingCSVWriter;

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

  private PeriodicIO mPeriodicIO;
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
  private DriveMotionPlanner mMotionPlanner;
  private DriveControlState mDriveControlState;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  private boolean mOverrideTrajectory = false;

  private boolean mIsBrakeMode;

  private static final int kLowGearVelocityControlSlot = 0;
  private static final double DRIVE_ENCODER_PPR = 4096.0;

  public Drivetrain() {

    super();

    mPeriodicIO = new PeriodicIO();

    setFactoryDefault();
    setBrakeMode(false);
    configureMotors();
    setMotorSafeties();
    setPigeonStatusFrame();
    
    m_drive = new DifferentialDrive(m_driveLeft1, m_driveRight1);
    m_drive.setSafetyEnabled(false);

    mMotionPlanner = new DriveMotionPlanner();
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

  private void setFactoryDefault() {
    m_driveLeft1.configFactoryDefault();
    m_driveLeft2.configFactoryDefault();
    m_driveLeft3.configFactoryDefault();
    m_driveRight1.configFactoryDefault();
    m_driveRight2.configFactoryDefault();
    m_driveRight3.configFactoryDefault();
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

  private static double rotationsToInches(double rotations) {
    return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double rpmToInchesPerSecond(double rpm) {
      return rotationsToInches(rpm) / 60;
  }

  private static double inchesToRotations(double inches) {
      return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
      return inchesToRotations(inches_per_second) * 60;
  }

  private static double radiansPerSecondToTicksPer100ms(double rad_s) {
      return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
  }

  /**
   * Configure talons for open loop control
   */
  public synchronized void setOpenLoop(DriveSignal signal) {
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
          setBrakeMode(false);

          System.out.println("Switching to open loop");
          System.out.println(signal);
          mDriveControlState = DriveControlState.OPEN_LOOP;
      }
      mPeriodicIO.left_demand = signal.getLeft();
      mPeriodicIO.right_demand = signal.getRight();
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
  }

  /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
      if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
          // We entered a velocity control state.
          setBrakeMode(true);
          m_driveLeft1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
          m_driveRight1.selectProfileSlot(kLowGearVelocityControlSlot, 0);

          mDriveControlState = DriveControlState.PATH_FOLLOWING;
      }
      mPeriodicIO.left_demand = signal.getLeft();
      mPeriodicIO.right_demand = signal.getRight();
      mPeriodicIO.left_feedforward = feedforward.getLeft();
      mPeriodicIO.right_feedforward = feedforward.getRight();
  }

  public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
      if (mMotionPlanner != null) {
          mOverrideTrajectory = false;
          mMotionPlanner.reset();
          mMotionPlanner.setTrajectory(trajectory);
          mDriveControlState = DriveControlState.PATH_FOLLOWING;
      }
  }

  public boolean isDoneWithTrajectory() {
      if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
          return false;
      }
      return mMotionPlanner.isDone() || mOverrideTrajectory;
  }

  public boolean isBrakeMode() {
      return mIsBrakeMode;
  }

  public synchronized void setBrakeMode(boolean on) {
      if (mIsBrakeMode != on) {
          mIsBrakeMode = on;
          NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
          m_driveLeft1.setNeutralMode(mode);
          m_driveLeft2.setNeutralMode(mode);
          m_driveLeft3.setNeutralMode(mode);

          m_driveRight1.setNeutralMode(mode);
          m_driveRight2.setNeutralMode(mode);
          m_driveRight3.setNeutralMode(mode);
      }
  }

  public synchronized Rotation2d getHeading() {
      return mPeriodicIO.gyro_heading;
  }

  public synchronized void setHeading(Rotation2d heading) {
      System.out.println("SET HEADING: " + heading.getDegrees());

      mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(m_driveGyro.getFusedHeading()).inverse());
      System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

      mPeriodicIO.gyro_heading = heading;
  }

  
  public synchronized void stop() {
      setOpenLoop(DriveSignal.NEUTRAL);
  }

  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
    SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
    SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
    SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
    SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
    SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

    SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
    SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
    SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());
    if (getHeading() != null) {
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    }
    if (mCSVWriter != null) {
        mCSVWriter.write();
    }
  }

  public synchronized void resetEncoders() {
      m_driveLeft1.setSelectedSensorPosition(0, 0, 0);
      m_driveRight1.setSelectedSensorPosition(0, 0, 0);
      mPeriodicIO = new PeriodicIO();
  }

  public void zeroSensors() {
      setHeading(Rotation2d.identity());
      resetEncoders();
  }

  public double getLeftEncoderRotations() {
      return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getRightEncoderRotations() {
      return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
  }

  public double getLeftEncoderDistance() {
      return rotationsToInches(getLeftEncoderRotations());
  }

  public double getRightEncoderDistance() {
      return rotationsToInches(getRightEncoderRotations());
  }

  public double getRightVelocityNativeUnits() {
      return mPeriodicIO.right_velocity_ticks_per_100ms;
  }

  public double getRightLinearVelocity() {
      return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLeftVelocityNativeUnits() {
      return mPeriodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
      return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLinearVelocity() {
      return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
      return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
  }

  public void overrideTrajectory(boolean value) {
      mOverrideTrajectory = value;
  }

  private void updatePathFollower() {
      if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
          final double now = Timer.getFPGATimestamp();

          DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

          // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

          mPeriodicIO.error = mMotionPlanner.error();
          mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

          if (!mOverrideTrajectory) {
              setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                      new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

              mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
              mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
          } else {
              setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
              mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
          }
      } else {
          DriverStation.reportError("Drive is not in path following state", false);
      }
  }

  public synchronized void reloadGains() {
      m_driveLeft1.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

      m_driveRight1.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
  }
  public void writeToLog() {
  }

  public synchronized void readPeriodicInputs() {
      double prevLeftTicks = mPeriodicIO.left_position_ticks;
      double prevRightTicks = mPeriodicIO.right_position_ticks;
      mPeriodicIO.left_position_ticks = m_driveLeft1.getSelectedSensorPosition(0);
      mPeriodicIO.right_position_ticks = m_driveRight1.getSelectedSensorPosition(0);
      mPeriodicIO.left_velocity_ticks_per_100ms = m_driveLeft1.getSelectedSensorVelocity(0);
      mPeriodicIO.right_velocity_ticks_per_100ms = m_driveRight1.getSelectedSensorVelocity(0);
      mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(m_driveGyro.getFusedHeading()).rotateBy(mGyroOffset);

      double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
      if (deltaLeftTicks > 0.0) {
          mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
      } else {
          mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
      }

      double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
      if (deltaRightTicks > 0.0) {
          mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
      } else {
          mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
      }

      if (mCSVWriter != null) {
          mCSVWriter.add(mPeriodicIO);
      }

      // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
  }

  public synchronized void writePeriodicOutputs() {
      if (mDriveControlState == DriveControlState.OPEN_LOOP) {
          m_driveLeft1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
          m_driveRight1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
      } else {
          m_driveLeft1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                  mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
          m_driveRight1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                  mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
      }
  }

  public synchronized void startLogging() {
      if (mCSVWriter == null) {
          mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
      }
  }

  public synchronized void stopLogging() {
      if (mCSVWriter != null) {
          mCSVWriter.flush();
          mCSVWriter = null;
      }
  }

  // The robot drivetrain's various states.
  public enum DriveControlState {
      OPEN_LOOP, // open loop voltage control
      PATH_FOLLOWING, // velocity PID control
  }

  public enum ShifterState {
      FORCE_LOW_GEAR,
      FORCE_HIGH_GEAR,
      AUTO_SHIFT
  }

  public static class PeriodicIO {
    // INPUTS
    public int left_position_ticks;
    public int right_position_ticks;
    public double left_distance;
    public double right_distance;
    public int left_velocity_ticks_per_100ms;
    public int right_velocity_ticks_per_100ms;
    public Rotation2d gyro_heading = Rotation2d.identity();
    public Pose2d error = Pose2d.identity();

    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_accel;
    public double right_accel;
    public double left_feedforward;
    public double right_feedforward;
    public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
  }
}
