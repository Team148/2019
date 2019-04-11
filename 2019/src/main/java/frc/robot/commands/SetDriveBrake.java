package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Add your docs here.
 */
public class SetDriveBrake extends InstantCommand {
  /**
   * Add your docs here.
   */
  public SetDriveBrake() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Drivetrain.getInstance().setBrakeMode(true);
  }

}
