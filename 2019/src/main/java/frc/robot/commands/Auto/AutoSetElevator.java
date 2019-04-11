package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class AutoSetElevator extends Command {

  private boolean m_isFinished = false;
  private double m_position;
  private double m_timeToWait;
  private double m_startTime = 0;
  private double m_tolerance;

  public AutoSetElevator(int position, double timeToWait) {
    requires(Elevator.getInstance());

    m_isFinished = false;

    m_position = position;
    m_timeToWait = timeToWait;
    m_tolerance = Constants.ELEVATOR_ERROR_TOLERANCE;

  }

  public AutoSetElevator(int position, double timeToWait, double tolerance) {
    requires(Elevator.getInstance());

    m_isFinished = false;

    m_position = position;
    m_timeToWait = timeToWait;
    m_tolerance = tolerance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(1.5 + m_timeToWait);
    m_isFinished = false;
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - m_startTime;
    double currentPosition = Elevator.getInstance().getElevatorPosition();
    double positionError = m_position - currentPosition;

    double slope = (Constants.ELEVATOR_F_UP - Constants.ELEVATOR_ZERO_F) / (Constants.ELEVATOR_ZERO_NEUTRAL_POSITION - Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND);
    double y_intercept = Constants.ELEVATOR_ZERO_F - (slope * Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND);
    double linear_F = slope * (Elevator.getInstance().getElevatorPosition()) + y_intercept;

    if (elapsedTime >= m_timeToWait) {
      if (Math.abs(positionError) < m_tolerance) {
        m_isFinished = true;
      }
      if (m_isFinished == false) {
        if (m_position > Constants.ELEVATOR_ZERO) {
          Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_F_UP);
        }

        if (Elevator.getInstance().getElevatorPosition() > Constants.ELEVATOR_ZERO_NEUTRAL_POSITION) {
          Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_F_UP);
        }
        else {
          if (Elevator.getInstance().getElevatorPosition() < Constants.ELEVATOR_ZERO_NEUTRAL_POSITION_DEADBAND) {
            Elevator.getInstance().setElevatorPosition(m_position, Constants.ELEVATOR_ZERO_F);
          }
          Elevator.getInstance().setElevatorPosition(m_position, linear_F);
        }
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_isFinished || isTimedOut();
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
