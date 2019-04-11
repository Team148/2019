package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;

public class HangStageTwo extends CommandGroup {

  public HangStageTwo() {
    addSequential(new SetForks(true));
  }
}
