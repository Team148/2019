package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import frc.robot.subsystems.EndGame;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {

  private static Elevator m_instance;

  private boolean m_isClosedLoop = false;
  private boolean m_isClosedLoopEndGame = false;
  private double m_position = 1.0;

  //Declare Elevator TalonSRXs
  private final WPI_TalonSRX m_elevator1 = new WPI_TalonSRX(RobotMap.ELEVATOR_ONE);
  private final WPI_TalonSRX m_elevator2 = new WPI_TalonSRX(RobotMap.ELEVATOR_TWO);
  private final WPI_TalonSRX m_elevator3 = new WPI_TalonSRX(RobotMap.ELEVATOR_THREE);

  //declare Elevator Shifter Solenoids
  private final DoubleSolenoid m_elevatorShifter = new DoubleSolenoid(0,RobotMap.ENDGAME_SHIFT_REVERSE, RobotMap.ENDGAME_SHIFT_FORWARD);

  public Elevator() {

    super();

    setFactoryDefault();
    setFollower();
    setBrakeMode(true);
    configureMotors();
    setMotorSafeties();
    setElevatorShifter(true);

  }
  @Override
  public void initDefaultCommand() {
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
    m_elevator2.configFactoryDefault();
    m_elevator3.configFactoryDefault();
  }

  private void setFollower() {
    m_elevator2.follow(m_elevator1);
    m_elevator3.follow(m_elevator1);
    m_elevator2.configOpenloopRamp(0.0, 0);
    m_elevator3.configOpenloopRamp(0.0, 0);
  }

  private void setBrakeMode(boolean mode) {
    if (mode == true) {
      m_elevator1.setNeutralMode(NeutralMode.Brake);
      m_elevator2.setNeutralMode(NeutralMode.Brake);
      m_elevator3.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_elevator1.setNeutralMode(NeutralMode.Coast);
      m_elevator2.setNeutralMode(NeutralMode.Coast);
      m_elevator3.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setMotorSafeties() {
    m_elevator1.setSafetyEnabled(false);
    m_elevator2.setSafetyEnabled(false);
    m_elevator3.setSafetyEnabled(false);
  }

  private void configureMotors() {
    m_elevator1.setInverted(false);
    m_elevator1.configOpenloopRamp(1.0, 0);
    m_elevator1.overrideLimitSwitchesEnable(false);
    m_elevator1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_elevator1.setSensorPhase(true); 
    m_elevator1.setSelectedSensorPosition(0, 0, 0);
    
  }

  public void joystickControl(double yStick) {
    m_elevator1.set(ControlMode.PercentOutput, yStick * Constants.ELEVATOR_OUTPUT_PERCENT);
  }

  public void configOpenLoop() {

    System.out.println("configOpenLoop");

    setFactoryDefault();
    m_elevator1.configVoltageCompSaturation(8.0, 0);
    m_elevator1.enableVoltageCompensation(true);

    m_isClosedLoop = false;
    m_isClosedLoopEndGame = false;

  }

  public void configClosedLoop() {

    System.out.println("configClosedLoop");

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
    m_isClosedLoopEndGame = false;
  }

  public void configClosedLoopEndGame() {

    System.out.println("configClosedLoopEndGame");

    m_elevator1.configVoltageCompSaturation(10.0, 0);
    m_elevator1.enableVoltageCompensation(true);

    m_elevator1.configNominalOutputForward(0.0, 0);
    m_elevator1.configNominalOutputReverse(0.0, 0);

    m_elevator1.configPeakOutputForward(Constants.ENDGAME_UP_OUTPUT_PERCENT, 0);
    m_elevator1.configPeakOutputReverse(Constants.ENDGAME_DOWN_OUTPUT_PERCENT, 0);

    m_elevator1.set(ControlMode.Position, 0.0);
    m_elevator1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    m_elevator1.configClosedloopRamp(0.10, 0);

    m_elevator1.config_kF(0, 0, 0);
    m_elevator1.config_kP(0, Constants.ENDGAME_P, 0);
    m_elevator1.config_kI(0, Constants.ENDGAME_I, 0);
    m_elevator1.config_kD(0, Constants.ENDGAME_D, 0);

    m_elevator1.setSelectedSensorPosition(0, 0, 0);

    m_isClosedLoop = false;
    m_isClosedLoopEndGame = true;
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

  public boolean isClosedLoopEndGame() {
    return m_isClosedLoopEndGame;
  }

  public int getElevatorPosition() {

    return m_elevator1.getSelectedSensorPosition(0);
  }

  public double getElevatorMPosition() {
    return m_position;
  }

  public int getElevatorVelocity() {

    return m_elevator1.getSelectedSensorVelocity(0);
  }

  public void setPositionManual(double position, double feedforward) {
    if (!m_isClosedLoop) {
      configClosedLoop();
    }

    m_position += position;

    m_elevator1.set(ControlMode.Position, m_position, DemandType.ArbitraryFeedForward, feedforward);
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

  public void setEndGamePosition(double position, double feedforward) {

    if (!m_isClosedLoopEndGame) {
      configClosedLoopEndGame();
    }



    m_position = position;

    //logic checks to protect the robot 
    int highest_position = Constants.ENDGAME_ZERO;
    if(EndGame.getInstance().getAnklesReleased()){
      highest_position = Constants.ENDGAME_ZERO_ANKLES;
    } //end logic check

    if (m_position > highest_position) {
      m_position = highest_position;
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

  public void setElevatorShifter (boolean isElevator) {


    if (isElevator) {
      m_elevatorShifter.set(Value.kForward);
    }
    else {
      m_elevatorShifter.set(Value.kReverse);
    }
  }

  public void setElevatorOpenLoop(double speed, boolean inverted)
  {

    double m_speed = speed;

    if(inverted)
      m_speed = -m_speed;
    if((isClosedLoop() || isClosedLoopEndGame())){
        configOpenLoop();

    m_elevator1.set(ControlMode.PercentOutput, m_speed);
    System.out.println("m_speed: " + m_speed);
    //System.out.println("setting open loop");
    }
  }
}
