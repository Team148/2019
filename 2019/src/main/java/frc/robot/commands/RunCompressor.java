/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.subsystems.Pneumatics;

public class RunCompressor extends Command {

  double m_position;
  boolean m_isFinished;

  public RunCompressor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Pneumatics.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if(Pneumatics.getInstance().comp.getPressureSwitchValue()) {
          Pneumatics.getInstance().setCompressor(false);
      }
      else {
          Pneumatics.getInstance().setCompressor(true);
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
      Pneumatics.getInstance().setCompressor(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}