package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;



public class SetElevatorShifter extends Command {

  private boolean m_isElevator;


  public SetElevatorShifter(boolean isElevator) {
    requires(Elevator.getInstance());
    System.out.println("Constructing Elevator Shifter Command");
    m_isElevator = isElevator;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.5);
    Elevator.getInstance().setElevatorShifter(m_isElevator);
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

    System.out.println("Finished Elevator Shifter Command");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
