
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;

public class HangStageThree extends CommandGroup {

  public HangStageThree() {
    addSequential(new SetAnkle(true));
  }
}
