// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.command.Subsystem;
// import frc.robot.RobotMap;

// /**
//  * Add your docs here.
//  */
// public class Wedge extends Subsystem {
//   // Put methods for controlling this subsystem
//   // here. Call these from Commands.

//   private static Wedge m_instance;

//   private final Solenoid m_wedge = new Solenoid(RobotMap.WEDGE_SOLENOID);

//   public Wedge() {

//     super();

//   }

//   @Override
//   public void initDefaultCommand() {
//     // Set the default command for a subsystem here.
//     // setDefaultCommand(new MySpecialCommand());
//   }

//   public static Wedge getInstance() { 
//     if (m_instance == null) {
//       m_instance = new Wedge();
//     }
//     return m_instance;
//   }

//   public void setWedge (boolean on) {
//     if (on) {
//       m_wedge.set(true);
//     }
//     else {
//       m_wedge.set(false);
//     }
//   }
// }