/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.FloorBallIntake;

/**
 * Add your docs here.
 */
public class RunBallFloorIntake extends InstantCommand {
  /**
   * Add your docs here.
   */

  private double m_percent;
  
  public RunBallFloorIntake(double percent) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(FloorBallIntake.getInstance());

    m_percent = percent;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    FloorBallIntake.getInstance().setBallIntakeMotor(m_percent);
  }

}
