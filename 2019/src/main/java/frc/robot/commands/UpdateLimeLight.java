package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Limelight;

public class UpdateLimeLight extends Command {
  public UpdateLimeLight() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Limelight.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    frc.robot.subsystems.Limelight.getInstance().setLimelightData();
    frc.robot.subsystems.Limelight.getInstance().getLimelightData();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}