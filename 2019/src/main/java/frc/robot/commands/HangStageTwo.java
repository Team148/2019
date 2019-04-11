package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class HangStageTwo extends CommandGroup {

  public HangStageTwo() {
    addSequential(new SetForks(true));
  }
}
