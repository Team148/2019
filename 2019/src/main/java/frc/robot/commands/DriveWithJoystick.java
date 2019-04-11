// package frc.robot.commands;

// import frc.robot.OI;
// import frc.robot.Robot;
// import frc.robot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj.command.Command;

// public class DriveWithJoystick extends Command {
//   public DriveWithJoystick() {
//     requires(Drivetrain.getInstance());
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     Robot.m_driveTrain.arcadeMode(OI.getInstance().m_driveJoystick.getRawAxis(4), OI.getInstance().m_driveJoystick.getRawAxis(1) * -1);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }
