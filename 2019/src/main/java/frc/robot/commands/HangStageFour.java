package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;

public class HangStageFour extends CommandGroup {

  public HangStageFour() {
    addSequential(new SetEndGameHeight(Constants.ENDGAME_CHILL));
    addSequential(new WaitCommand(0.25));
    addSequential(new SetEndGameHeight(Constants.ENDGAME_CHILL2));
  }
}
