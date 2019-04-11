package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.EndGame;



public class EndGameDrive extends Command {

  private double m_speed;
  private double m_timeout;


  public EndGameDrive(double speed, double timeout) {
    requires(EndGame.getInstance());
    System.out.println("Constructing EndGameDrive Command");
    m_speed = speed;
    m_timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(m_timeout);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    EndGame.getInstance().setEndGameDriveSpeed(m_speed);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    EndGame.getInstance().setEndGameDriveSpeed(0.0);
    System.out.println("Finished EndGameDrive Command");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
