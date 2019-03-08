
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;

public class HangStageTwo extends CommandGroup {

  public HangStageTwo() {
    addSequential(new EndGameDrive(0.5, 5.0));
  }
}
