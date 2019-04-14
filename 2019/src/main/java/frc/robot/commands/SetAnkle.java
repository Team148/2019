
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.EndGame;



public class SetAnkle extends Command {

  private boolean m_isBroken;


  public SetAnkle(boolean isBroken) {
    requires(EndGame.getInstance());
    System.out.println("Constructing Broken Ankles Command");
    m_isBroken = isBroken;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.5);
    EndGame.getInstance().setAnklesReleased(m_isBroken);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    System.out.println("Finished AnklesBroken Command");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
