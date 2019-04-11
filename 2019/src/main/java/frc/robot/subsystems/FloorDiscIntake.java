// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import frc.robot.RobotMap;

// /**
//  * Add your docs here.
//  */
// public class FloorDiscIntake extends Subsystem {

//   private static FloorDiscIntake m_instance;

//   private final WPI_TalonSRX m_Disc1 = new WPI_TalonSRX(RobotMap.DISC_INTAKE);

//   private final Solenoid m_discIntakeSolenoid = new Solenoid(RobotMap.DISC_INTAKE_SOLENOID);


  

//   public FloorDiscIntake() {

//     super();

//     setFactoryDefault();
//     setBrakeMode(true);
//     configureMotor();
//     setMotorSafeties();

//   }

//   @Override
//   public void initDefaultCommand() {
//     // setDefaultCommand(new MySpecialCommand());
//   }

//   public static FloorDiscIntake getInstance() {
//     if (m_instance == null) {
//       m_instance = new FloorDiscIntake();
//     }
//     return m_instance;
//   }

//   public void setFactoryDefault(){
//     m_Disc1.configFactoryDefault();
//   }

//   public void setBrakeMode(boolean mode){
//     if (mode == true) {
//       m_Disc1.setNeutralMode(NeutralMode.Brake);
//     }
//     else {
//       m_Disc1.setNeutralMode(NeutralMode.Coast);
//     }
//   }

//   public void configureMotor(){
//     m_Disc1.configOpenloopRamp(1.0, 0);
//   }
  
//   public void setMotorSafeties(){
//     m_Disc1.setSafetyEnabled(false);
//   }

//   public void setDiscIntakeMotor(double percent) {

//     m_Disc1.set(ControlMode.PercentOutput, percent);
//   }

//   public double getAverageCurrent() {

//     return m_Disc1.getOutputCurrent();
//   }

//   public void setDiscIntakeCylinder (boolean on) {
//     if (on) {
//       m_discIntakeSolenoid.set(true);
//     }
//     else {
//       m_discIntakeSolenoid.set(false);
//     }
//   }
// }