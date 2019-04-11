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
    requires(FloorBallIntake.getInstance());

    m_percent = percent;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    FloorBallIntake.getInstance().setBallIntakeMotor(m_percent);
  }

}
