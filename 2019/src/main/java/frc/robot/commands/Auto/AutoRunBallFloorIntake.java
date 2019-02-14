/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.FloorBallIntake;

public class AutoRunBallFloorIntake extends Command {

  private boolean m_isIntakeOn = false;
  private boolean m_isFinished = false;
  private boolean m_startToIntakeBall = false;

  private int m_timesExecuted;
  private double m_totalCurrent;
  private double m_percent;
  private double m_runTime;
  private double m_startTime;
  private double m_currentTime;

  public AutoRunBallFloorIntake(double percent, double runTime) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(FloorBallIntake.getInstance());

    m_percent = percent;
    m_runTime = runTime;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    FloorBallIntake.getInstance().setFloorIntakeMotor(m_percent);

    m_totalCurrent = 0;
    m_timesExecuted = 0;
    m_startTime = Timer.getFPGATimestamp();

    m_isIntakeOn = true;
    m_isFinished = false;

    setTimeout(m_runTime);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    FloorBallIntake.getInstance().setFloorIntakeMotor(m_percent);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return m_isFinished || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    FloorBallIntake.getInstance().setFloorIntakeMotor(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
