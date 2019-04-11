/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Forks;



public class SetForks extends Command {

  private boolean m_isSet;


  public SetForks(boolean set) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Forks.getInstance());
    System.out.println("Constructing Broken Ankles Command");
    m_isSet = set;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.5);
    Forks.getInstance().setForks(m_isSet);
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

    System.out.println("Finished SetForks Command");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
