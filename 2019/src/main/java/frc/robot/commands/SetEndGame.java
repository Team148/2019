/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndGame;
import frc.robot.Constants;
import frc.robot.OI;

public class SetEndGame extends Command {

  double m_position;
  boolean m_isFinished;

  public SetEndGame(double position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Elevator.getInstance());
    //requires(EndGame.getInstance());

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

    if(abs(Elevator.getInstance().getElevatorPosition()-m_position) < (double)(Constants.ENDGAME_FINISH_TOLERANCE)){
      m_isFinished = true;
      break;
    }
    else
      Elevator.getInstance().setEndGamePosition(m_position, Constants.ENDGAME_F_UP);

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
