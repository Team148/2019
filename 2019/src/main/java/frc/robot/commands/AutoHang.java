package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Constants;


public class AutoHang extends CommandGroup {

    public  AutoHang() {
        addSequential(new SetElevatorShifter(false));
        addSequential(new WaitCommand(2.0));
        addSequential(new SetEndGameHeight(Constants.ENDGAME_TOP));
        addSequential(new WaitCommand(2.0));
        addSequential(new EndGameDrive(0.5, 5.0));
        addSequential(new WaitCommand(2.0));
        addSequential(new SetAnkle(true));
        addSequential(new WaitCommand(2.0));
        addSequential(new EndGameDrive(0.5, 5.0));
        addSequential(new WaitCommand(2.0));
        addSequential(new SetDriveBrake());
        addSequential(new SetEndGameHeight(Constants.ENDGAME_CHILL));
        addSequential(new WaitCommand(0.25));
        addSequential(new SetEndGameHeight(Constants.ENDGAME_CHILL2));
        addSequential(new WaitCommand(2.0));
    }
}