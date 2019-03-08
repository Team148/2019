
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;

public class HangStageOne extends CommandGroup {

  public HangStageOne() {
    addSequential(new SetElevatorShifter(false));
    addSequential(new SetEndGameHeight(Constants.ENDGAME_TOP));
    addSequential(new SetDriveBrake());
  }
}
