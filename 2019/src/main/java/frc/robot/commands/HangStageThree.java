package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class HangStageThree extends CommandGroup {

  public HangStageThree() {
    addSequential(new SetAnkle(true));
  }
}
