package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class EndGame extends Subsystem {

  private static EndGame m_instance;


  private boolean m_AnklesReleased = false;


  private final DigitalInput m_mexicoSensor = new DigitalInput(RobotMap.MEXICO_SENSOR_ONE);
  private final DoubleSolenoid m_endGameAnkles = new DoubleSolenoid(0, RobotMap.ENDGAME_FEET_FORWARD, RobotMap.ENDGAME_FEET_REVERSE);
  private final WPI_TalonSRX m_EndGameDrive = new WPI_TalonSRX(RobotMap.ENDGAME_DRIVE);

  public EndGame() {

    super();

    this.configureMotors();
    this.setAnklesReleased(false);

  }

  @Override
  public void initDefaultCommand() {
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

    setFactoryDefault();    

    m_EndGameDrive.set(ControlMode.PercentOutput, 0.0);

    m_EndGameDrive.configOpenloopRamp(0.5, 0);

    m_EndGameDrive.configVoltageCompSaturation(10.0, 0);
    m_EndGameDrive.enableVoltageCompensation(true);

    m_EndGameDrive.configNominalOutputForward(0.0, 0);
    m_EndGameDrive.configNominalOutputReverse(0.0, 0);
    m_EndGameDrive.setInverted(true);
    
  }


  public void setEndGameDriveSpeed(double speed){

    //note only positive values

      m_EndGameDrive.set(ControlMode.PercentOutput, speed);
    // if(on)
    //   m_EndGameDrive.set(ControlMode.PercentOutput, -Constants.ENDGAME_DRIVE_SPEED);
    // else
    //   m_EndGameDrive.set(ControlMode.PercentOutput, 0);
  }

  public void setAnklesReleased (boolean isBroken) {

    if (isBroken) {
      m_endGameAnkles.set(Value.kForward);
      m_AnklesReleased = true;
    }
    else {
      m_endGameAnkles.set(Value.kReverse);
      m_AnklesReleased = false;
    }
  }

  public boolean getAnklesReleased(){
    return m_AnklesReleased;
  }

  public boolean getMexicoSensor(){
    return m_mexicoSensor.get();
  }

}
