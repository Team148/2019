package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class SetElevator extends Command {

  double m_position;
  boolean m_isFinished;

  public SetElevator(double position) {
    requires(Elevator.getInstance());

    m_position = position;
    m_isFinished = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_isFinished = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double slope = (Constants.ELEVATOR_F_DOWN - Constants.ELEVATOR_ZERO_F) / (Constants.ELEVATOR_ZERO_NEUTRAL_POSITION - Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND);
    double yIntercept = Constants.ELEVATOR_ZERO_F - (slope * Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND);
    double linearF = slope * (Elevator.getInstance().getElevatorPosition()) + yIntercept;

    if (m_position > Constants.ELEVATOR_ZERO) {
      Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_F_UP);
      m_isFinished = true;
    }

    if (m_isFinished == false) {
      if (Elevator.getInstance().getElevatorPosition() > Constants.ELEVATOR_ZERO_NEUTRAL_POSITION) {
        Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_F_DOWN);
      }
      else {
        if (Elevator.getInstance().getElevatorPosition() < Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND) {
          Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_ZERO_F);
          m_isFinished = true;
        }
        Elevator.getInstance().setElevatorPosition(m_position, linearF);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
