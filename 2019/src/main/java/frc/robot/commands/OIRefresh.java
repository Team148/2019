// package frc.robot.commands;

// import frc.robot.subsystems.Elevator;
// import frc.robot.Constants;
// import frc.robot.OI;
// import frc.robot.RobotMap;
// import edu.wpi.first.wpilibj.command.Command;

// public class OIRefresh extends Command {
//     public OIRefresh() {
//     }
  
//     // Called just before this Command runs the first time
//     @Override
//     protected void initialize() {
//     }
  
//     // Called repeatedly when this Command is scheduled to run
//     @Override
//     protected void execute() {
//       if(Math.abs(OI.getInstance().m_operatorJoystick.getRawAxis(1)) > Constants.ELEVATOR_MANUAL_DEADBAND) {
//           double rawAxis1 = -OI.getInstance().m_operatorJoystick.getRawAxis(1);

//           if(rawAxis1 > 1) {
//               rawAxis1 = 1;
//           }
//           if(rawAxis1 < -1) {
//               rawAxis1 = -1;
//           }

//           double elevatorIncrement = rawAxis1 * Constants.ELEVATOR_MANUAL_DPOS_SCALAR * Constants.TELE_PERIODIC_DT;

//           if(elevatorIncrement != 0.0) {
//               Elevator.getInstance().incrementElevatorPosition(elevatorIncrement);
//           }
//       }
//     }
  
//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     protected boolean isFinished() {
//       return false;
//     }
  
//     // Called once after isFinished returns true
//     @Override
//     protected void end() {
//       Elevator.getInstance().joystickControl(0.0);
//     }
  
//     // Called when another command which requires one or more of the same
//     // subsystems is scheduled to run
//     @Override
//     protected void interrupted() {
//     }
//   }