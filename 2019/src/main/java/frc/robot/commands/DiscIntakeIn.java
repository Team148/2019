// package frc.robot.commands;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.subsystems.FloorDiscIntake;

// public class DiscIntakeIn extends Command {
//   public DiscIntakeIn() {
//     requires(FloorDiscIntake.getInstance());
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     FloorDiscIntake.getInstance().setDiscIntakeCylinder(false);
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