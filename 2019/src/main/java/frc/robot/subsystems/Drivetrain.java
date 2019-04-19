package frc.robot.subsystems;

import frc.robot.subsystems.Subsystem;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.RobotMap;
import frc.robot.RobotState;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.planners.DriveMotionPlanner;

import lib.geometry.Pose2d;
import lib.geometry.Pose2dWithCurvature;
import lib.geometry.Rotation2d;
import lib.geometry.Twist2d;
import lib.trajectory.TrajectoryIterator;
import lib.trajectory.timing.TimedState;

import lib.util.DriveSignal;
import lib.util.ReflectingCSVWriter;

import frc.loops.ILooper;
import frc.loops.Loop;

public class Drivetrain extends Subsystem {

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

  private DifferentialDrive m_drive;
  private RobotState mRobotState = RobotState.getInstance();
  private PeriodicIO mPeriodicIO;
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
  private DriveMotionPlanner mMotionPlanner;
  private DriveControlState mDriveControlState;
  private TalonControlState mTalonControlState;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  private Rotation2d mTargetHeading = new Rotation2d();
  private boolean mOverrideTrajectory = false;
  private boolean mIsBrakeMode;
  private boolean mIsOnTarget = false;

  private int mInitLeftPosition;
  private int mInitRightPosition;
  private int mInitLeftPositionCorrection;
  private int mInitRightPositionCorrection;
  private int mInitTicksNeeded;


  private static final int kLowGearVelocityControlSlot = 0;
  private static final int kLowGearPositionControlSlot = 1;
  private static final double DRIVE_ENCODER_PPR = 6956.0;


  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timestamp) {
        synchronized (Drivetrain.this) {
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            // setOpenLoop(new DriveSignal(0.05, 0.05));
            // setBrakeMode(false);
//                 startLogging();
        }
    }

    @Override
    public void onLoop(double timestamp) {

        
        synchronized (Drivetrain.this) {
            switch (mDriveControlState) {
                case OPEN_LOOP:
                    break;
                case PATH_FOLLOWING:
                    updatePathFollower();
                    break;
                case TURN_TO_HEADING:
                    updateTurnToHeading(timestamp);
                    // System.out.println("TURN TO HEADING");
                    return;
                case DRIVE_VELOCITY:
                    break;
                default:
                    System.out.println("Unexpected drive control state: " + mDriveControlState);
                    break;
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
        stop();
        stopLogging();
    }
};

  public Drivetrain() {

    super();

    mPeriodicIO = new PeriodicIO();

    setFactoryDefault();
    setBrakeMode(false);
    configureMotors();
    setMotorSafeties();
    setPigeonStatusFrame();

    reloadGains();
    
    m_drive = new DifferentialDrive(m_driveLeft1, m_driveRight1);
    m_drive.setSafetyEnabled(false);

    mMotionPlanner = new DriveMotionPlanner();
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

    m_driveLeft1.enableVoltageCompensation(true);
    m_driveRight1.enableVoltageCompensation(true);

    m_driveLeft1.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
    m_driveRight1.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);

    m_driveLeft1.configNominalOutputForward(0.0, 0);
    m_driveRight2.configNominalOutputForward(0.0, 0);

    m_driveLeft1.configPeakOutputForward(1.0, 0);
    m_driveLeft1.configPeakOutputReverse(-1.0, 0);

    m_driveRight1.configPeakOutputForward(1.0, 0);
    m_driveRight1.configPeakOutputReverse(-1.0, 0);

    m_driveLeft1.overrideLimitSwitchesEnable(false);
    m_driveRight1.overrideLimitSwitchesEnable(false);
	
    m_driveLeft1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_driveRight1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    m_driveLeft1.setSensorPhase(true);
    //SET FOR COMPBOT PLZ
    m_driveRight1.setSensorPhase(false);
    // m_driveRight1.setSensorPhase(true);
  }


  private void configureOpenTalon(){
    setBrakeMode(false);

    System.out.println("Switching to open loop");

    mTalonControlState = TalonControlState.OPEN;
  }

  private void configureVelocityTalon(){

    setBrakeMode(true);
    m_driveLeft1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
    m_driveRight1.selectProfileSlot(kLowGearVelocityControlSlot, 0);

    m_driveLeft1.configClosedloopRamp(0);
    m_driveRight1.configClosedloopRamp(0);

    mTalonControlState = TalonControlState.VELOCITY;

    System.out.println("Switching to velocity");

  }

  private void configureMagicTalon(){

    setBrakeMode(true);
    m_driveLeft1.selectProfileSlot(kLowGearPositionControlSlot, 0);
    m_driveRight1.selectProfileSlot(kLowGearPositionControlSlot, 0);
    
    m_driveLeft1.configClosedloopRamp(12/Constants.kDriveLowGearPositionRampRate);
    m_driveRight1.configClosedloopRamp(12/Constants.kDriveLowGearPositionRampRate);

    mTalonControlState = TalonControlState.MOTION_MAGIC;

    System.out.println("Switching to Magic");
  }

  private void configurePositionTalon(){

    setBrakeMode(true);
    m_driveLeft1.selectProfileSlot(kLowGearPositionControlSlot, 0);
    m_driveRight1.selectProfileSlot(kLowGearPositionControlSlot, 0);
    
    m_driveLeft1.configClosedloopRamp(12/Constants.kDriveLowGearPositionRampRate);
    m_driveRight1.configClosedloopRamp(12/Constants.kDriveLowGearPositionRampRate);

    mTalonControlState = TalonControlState.POSITION;

    System.out.println("Switching to Position");
  }

  public void setMotorSafeties() {
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
      return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
  }

@Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

  /**
   * Configure talons for open loop control
   */
  public synchronized void setOpenLoop(DriveSignal signal) {
      if (mDriveControlState != DriveControlState.OPEN_LOOP) {
        //   setBrakeMode(false);

        //   System.out.println("Switching to open loop");
        // if(mTalonControlState != TalonControlState.OPEN){
        //     configureOpenTalon();
        // }
            
          System.out.println(signal);
          mDriveControlState = DriveControlState.OPEN_LOOP;
      }
      if(mTalonControlState != TalonControlState.OPEN){
        configureOpenTalon();
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
          
        //   if(mTalonControlState != TalonControlState.VELOCITY){
        //       configureVelocityTalon();
        //   }

        //   setBrakeMode(true);
        //   m_driveLeft1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
        //   m_driveRight1.selectProfileSlot(kLowGearVelocityControlSlot, 0);

        //   System.out.println("Switching to velocity");

          mDriveControlState = DriveControlState.PATH_FOLLOWING;
      }
      if(mTalonControlState != TalonControlState.VELOCITY){
        configureVelocityTalon();
    }

      mPeriodicIO.left_demand = signal.getLeft();
      mPeriodicIO.right_demand = signal.getRight();
      mPeriodicIO.left_feedforward = feedforward.getLeft();
      mPeriodicIO.right_feedforward = feedforward.getRight();
  }

  public synchronized void setPositionMagic(DriveSignal signal) {
    if (mDriveControlState != DriveControlState.TURN_TO_HEADING) {
        // We entered a velocity control state.
        // if(mTalonControlState != TalonControlState.MOTION_MAGIC){
        //     configureMagicTalon();
        // }
        // setBrakeMode(true);
        // m_driveLeft1.selectProfileSlot(kLowGearPositionControlSlot, 0);
        // m_driveRight1.selectProfileSlot(kLowGearPositionControlSlot, 0);

        // System.out.println("Switching to Position");

        mDriveControlState = DriveControlState.TURN_TO_HEADING;
    }
    if(mTalonControlState != TalonControlState.MOTION_MAGIC){
        configureMagicTalon();
    }

    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    // mPeriodicIO.left_feedforward = feedforward.getLeft();
    // mPeriodicIO.right_feedforward = feedforward.getRight();
}


public synchronized void setPosition(DriveSignal signal) {
    if(mTalonControlState != TalonControlState.POSITION){
        configurePositionTalon();
    }

    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    // mPeriodicIO.left_feedforward = feedforward.getLeft();
    // mPeriodicIO.right_feedforward = feedforward.getRight();
}

    /**
   * Configures talons for velocity control
   */
  public synchronized void setVelocityInchesPerSecond(DriveSignal signal) {
    if (!(mDriveControlState == DriveControlState.DRIVE_VELOCITY)) {
        // // We entered a velocity control state.
        // System.out.println("Switching to velocity inches");
        // setBrakeMode(true);
        // m_driveLeft1.selectProfileSlot(kLowGearVelocityControlSlot, 0);
        // m_driveRight1.selectProfileSlot(kLowGearVelocityControlSlot, 0);

        if(mTalonControlState != TalonControlState.VELOCITY){
            configureVelocityTalon();
        }
    }
    if (mDriveControlState != DriveControlState.DRIVE_VELOCITY){
        mDriveControlState = DriveControlState.DRIVE_VELOCITY;   
    }

    // SmartDashboard.putNumber("LeftIPS", signal.getLeft());
    // SmartDashboard.putNumber("RightIPS",signal.getRight());
    double left_velocity_rev = inchesToRotations(signal.getLeft())*(2*Math.PI);
    double right_velocity_rev = inchesToRotations(signal.getRight())*(2*Math.PI);

    mPeriodicIO.left_demand = radiansPerSecondToTicksPer100ms(left_velocity_rev);
    mPeriodicIO.right_demand =  radiansPerSecondToTicksPer100ms(right_velocity_rev);
    mPeriodicIO.left_feedforward = left_velocity_rev * Constants.kDriveKv;
    mPeriodicIO.right_feedforward = right_velocity_rev * Constants.kDriveKv;
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

    /**
     * Configures the drivebase to turn to a desired heading
     */
    public synchronized void setWantTurnToHeading(Rotation2d heading) {
       mDriveControlState = DriveControlState.TURN_TO_HEADING;
        // updatePositionSetpoint(getLeftEncoderDistance(), getRightEncoderDistance());
        mInitLeftPosition = mPeriodicIO.left_position_ticks;
        mInitRightPosition = mPeriodicIO.right_position_ticks;
        // setPositionMagic(new DriveSignal(mInitLeftPosition, mInitRightPosition));
        setPosition(new DriveSignal(mInitLeftPosition, mInitRightPosition));


        if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
            mTargetHeading = heading;
            mIsOnTarget = false;

            final Rotation2d field_to_robot = getHeading();
            final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

            Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));

            mInitTicksNeeded = (int)(inchesToRotations(wheel_delta.right) *DRIVE_ENCODER_PPR) ;

            mInitLeftPositionCorrection = (int)(mInitLeftPosition - mInitTicksNeeded);
            mInitRightPositionCorrection =  (int)(mInitRightPosition + mInitTicksNeeded);
        }
    }


    /**
     * Turn the robot to a target heading.
     * 
     * Is called periodically when the robot is auto-aiming towards the boiler.
     */
    private void updateTurnToHeading(double timestamp) {
        final Rotation2d field_to_robot = getHeading();
        // System.out.println("Running updateTurnToHeading");
        // final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
        
        // System.out.println("FIELD TO VEHICLE IS  " + field_to_robot.getDegrees());

        // Figure out the rotation necessary to turn to face the goal.


        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);
        

        // Check if we are on target
        final double kGoalPosTolerance = 2.0; // degrees
        final double kGoalVelTolerance = 5.0; // inches per second
        if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance && Math.abs(getLeftLinearVelocity()) < kGoalVelTolerance && Math.abs(getRightLinearVelocity()) < kGoalVelTolerance) {
            // if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance) {

            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            setPosition(new DriveSignal(mPeriodicIO.left_position_ticks, mPeriodicIO.right_position_ticks));

            return;
        }

        Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        int ticksNeeded = (int)(inchesToRotations(wheel_delta.right) *DRIVE_ENCODER_PPR ) ;

        // System.out.println("ticksNeeded: " + ticksNeeded);

        int m_l_cur_pos = mPeriodicIO.left_position_ticks;
        int m_r_cur_pos = mPeriodicIO.right_position_ticks;

        // System.out.println("curr_L " + m_l_cur_pos + " curr_R " + m_r_cur_pos);

        // int m_pos_traveled_l = ((mInitLeftPosition - m_l_cur_pos));//+ mInitLeftPosition);
        // int m_pos_traveled_r = ((mInitRightPosition + m_r_cur_pos));// - mInitRightPosition);
        // System.out.println("Traveled L: " + m_pos_traveled_l + " Traveled R: " + m_pos_traveled_r);
    
        // int m_pos_err_l_e = ((mInitTicksNeeded - (m_pos_traveled_l)));
        // int m_pos_err_r_e = ((mInitTicksNeeded + (m_pos_traveled_r)));
        // System.out.println("DeltaTicks L: " + m_pos_err_l_e + "DeltaTicks R: " + m_pos_err_r_e);
    
        // int m_pos_err_l = ticksNeeded - m_pos_err_l_e - mInitLeftPosition;
        // int m_pos_err_r = ticksNeeded - m_pos_err_r_e - mInitRightPosition;

        // int wantLeftPos = mInitLeftPositionCorrection - m_pos_err_l;
        // int wantRightPos = mInitRightPositionCorrection + m_pos_err_r;

             int wantLeftPos = m_l_cur_pos - ticksNeeded;
        int wantRightPos = m_r_cur_pos + ticksNeeded;



        //System.out.println("Theta Error: " + robot_to_target.getDegrees() );//+ " wdl: " + wheel_delta.left + " wdr: " + wheel_delta.right);
        // double wantLeftPos = inchesToRotations(wheel_delta.left) *DRIVE_ENCODER_PPR + mPeriodicIO.left_position_ticks;
        // double wantRightPos =  inchesToRotations(wheel_delta.right) * DRIVE_ENCODER_PPR  + mPeriodicIO.right_position_ticks;
        // System.out.println("L: " + wantLeftPos + " R: " +wantRightPos);
        // setPositionMagic(new DriveSignal(wantLeftPos, wantRightPos));

        setPosition(new DriveSignal(wantLeftPos, wantRightPos));
    }

    // public double getLeftVelocityInchesPerSec() {
    //     return rpmToInchesPerSecond(getLeftVelocityNativeUnits());
    // }

    // public double getRightVelocityInchesPerSec() {
    //     return rpmToInchesPerSecond(getRightVelocityNativeUnits());
    // }

    public synchronized boolean isDoneWithTurn() {
        if (mDriveControlState == DriveControlState.TURN_TO_HEADING) {
            return mIsOnTarget;
        } else {
            System.out.println("Robot is not in turn to heading mode");
            return false;
        }
    }

  @Override
  public synchronized void stop() {
      setOpenLoop(DriveSignal.NEUTRAL);
  }

  @Override
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

  @Override
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
      return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);   //Inches / second
  }

  public double getLeftVelocityNativeUnits() {
      return mPeriodicIO.left_velocity_ticks_per_100ms;
  }

  public double getLeftLinearVelocity() {
      return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);    //Inches / second
  }

  public double getLinearVelocity() {
      return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;        //Inches / second
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
                SmartDashboard.putNumber("LeftIPS", rotationsToInches(output.left_velocity/(Math.PI*2)));
                SmartDashboard.putNumber("RightIPS", rotationsToInches(output.right_velocity/(Math.PI*2)));
              mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
              mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
          } else {
              setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
              mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
          }
      } else {
          DriverStation.reportError("Drivetrain is not in path following state", false);
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

      m_driveLeft1.config_kP(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKp, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kI(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKi, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kD(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKd, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_kF(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKf, Constants.kLongCANTimeoutMs);
      m_driveLeft1.config_IntegralZone(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionIZone, Constants.kLongCANTimeoutMs);
        m_driveLeft1.configMotionCruiseVelocity((int)radiansPerSecondToTicksPer100ms(Constants.kDriveLowGearMaxVelocity*(2*Math.PI)/60));
        m_driveLeft1.configMotionAcceleration((int)radiansPerSecondToTicksPer100ms(Constants.kDriveLowGearMaxAccel*(2*Math.PI)/60));


      m_driveRight1.config_kP(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKp, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kI(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKi, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kD(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKd, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_kF(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionKf, Constants.kLongCANTimeoutMs);
      m_driveRight1.config_IntegralZone(kLowGearPositionControlSlot, Constants.kDriveLowGearPositionIZone, Constants.kLongCANTimeoutMs);
      m_driveRight1.configMotionCruiseVelocity((int)radiansPerSecondToTicksPer100ms(Constants.kDriveLowGearMaxVelocity*(2*Math.PI)/60));
      m_driveRight1.configMotionAcceleration((int)radiansPerSecondToTicksPer100ms(Constants.kDriveLowGearMaxAccel*(2*Math.PI)/60));

    }

  @Override
  public void writeToLog() {
  }

  @Override
  public synchronized void readPeriodicInputs() {
      double prevLeftTicks = mPeriodicIO.left_position_ticks;
      double prevRightTicks = mPeriodicIO.right_position_ticks;
      mPeriodicIO.left_position_ticks = m_driveLeft1.getSelectedSensorPosition(0);
      mPeriodicIO.right_position_ticks = m_driveRight1.getSelectedSensorPosition(0);
      mPeriodicIO.left_velocity_ticks_per_100ms = m_driveLeft1.getSelectedSensorVelocity(0);
      mPeriodicIO.right_velocity_ticks_per_100ms = m_driveRight1.getSelectedSensorVelocity(0);
      mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(m_driveGyro.getFusedHeading()).rotateBy(mGyroOffset);

      double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR) * Math.PI;
      if (deltaLeftTicks > 0.0) {
          mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
      } else {
          mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
      }

      double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / DRIVE_ENCODER_PPR) * Math.PI;
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

  @Override
  public synchronized void writePeriodicOutputs() {

    // System.out.println("Left Demand: " + mPeriodicIO.left_demand + "Left Arbitrary: " + mPeriodicIO.left_feedforward);
    //   if (mDriveControlState == DriveControlState.OPEN_LOOP) {
        if (mTalonControlState == TalonControlState.OPEN) {
          m_driveLeft1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
          m_driveRight1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
      }
    //         if(mDriveControlState == DriveControlState.TURN_TO_HEADING){
      if(mTalonControlState == TalonControlState.MOTION_MAGIC){
        m_driveLeft1.set(ControlMode.MotionMagic, (int)mPeriodicIO.left_demand);
        m_driveRight1.set(ControlMode.MotionMagic, (int)mPeriodicIO.right_demand);
      }
    //   else { //default case velocity
        if(mTalonControlState == TalonControlState.VELOCITY){
          m_driveLeft1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                  mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / (DRIVE_ENCODER_PPR/4.0));
          m_driveRight1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                  mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / (DRIVE_ENCODER_PPR/4.0));
      }
        if(mTalonControlState == TalonControlState.POSITION){
            m_driveLeft1.set(ControlMode.Position, mPeriodicIO.left_demand);
            m_driveRight1.set(ControlMode.Position, mPeriodicIO.right_demand);
        }
  }

  @Override public boolean checkSystem() {
      return true;
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
      TURN_TO_HEADING, // turn in place
      DRIVE_VELOCITY // drive velocity
  }

  //The robot Talon Control States
  public enum TalonControlState{
        OPEN,
        VELOCITY,       
        MOTION_MAGIC,
        POSITION
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
