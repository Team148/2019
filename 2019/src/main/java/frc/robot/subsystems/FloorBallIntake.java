package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class FloorBallIntake extends Subsystem {

  private static FloorBallIntake m_instance;

  private final WPI_TalonSRX m_Ball1 = new WPI_TalonSRX(RobotMap.BALL_INTAKE);

  private final Solenoid m_ballIntakeSolenoid = new Solenoid(RobotMap.BALL_INTAKE_SOLENOID);


  

  public FloorBallIntake() {

    super();

    setFactoryDefault();
    setBrakeMode(true);
    setMotorSafeties();

  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new MySpecialCommand());
  }

  public static FloorBallIntake getInstance() {
    if (m_instance == null) {
      m_instance = new FloorBallIntake();
    }
    return m_instance;
  }

  public void setFactoryDefault(){
    m_Ball1.configFactoryDefault();
  }

  public void setBrakeMode(boolean mode){
    if (mode == true) {
      m_Ball1.setNeutralMode(NeutralMode.Brake);
    }
    else {
      m_Ball1.setNeutralMode(NeutralMode.Coast);
    }
  }
  
  // public void configureMotor(){
  // }

  public void setMotorSafeties(){
    m_Ball1.setSafetyEnabled(false);
  }

  public void setBallIntakeMotor(double percent) {

    m_Ball1.set(ControlMode.PercentOutput, percent);
  }

  public double getAverageCurrent() {

    return m_Ball1.getOutputCurrent();
  }

  public void setBallIntakeCylinder (boolean on) {
    if (on) {
      m_ballIntakeSolenoid.set(true);
    }
    else {
      m_ballIntakeSolenoid.set(false);
    }
  }
}
