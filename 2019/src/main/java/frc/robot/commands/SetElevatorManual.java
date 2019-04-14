package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.OI;

public class SetElevatorManual extends Command {
  public SetElevatorManual() {
    requires(Elevator.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if((OI.getInstance().m_operatorJoystick.getRawAxis(1) > Constants.ELEVATOR_MANUAL_DEADBAND_UP) || (OI.getInstance().m_operatorJoystick.getRawAxis(1) < Constants.ELEVATOR_MANUAL_DEADBAND_DOWN)) {
      Elevator.getInstance().setPositionManual((OI.getInstance().m_operatorJoystick.getRawAxis(1) * Constants.ELEVATOR_MANUAL_DPOS_SCALAR), Constants.ELEVATOR_F_UP);
    }
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
