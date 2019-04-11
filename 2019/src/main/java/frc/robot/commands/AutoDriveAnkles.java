package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.EndGame;



public class AutoDriveAnkles extends Command {

  private boolean m_isBroken;

  private double m_initTime;
  private double m_minimumTime = 1;
  private double m_tripTime;
  private double m_timeAfterAnkles = 2;
  private boolean m_isFinished = false;

  private static EndGame m_EndGame;

  


  public AutoDriveAnkles(boolean isBroken) {
    requires(EndGame.getInstance());
    System.out.println("Constructing Auto; Ankles Command");
    m_isBroken = isBroken;
    m_EndGame = EndGame.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(0.5);
    m_initTime = Timer.getFPGATimestamp();
    m_isFinished = false;

    
    //EndGame.getInstance().setAnklesReleased(m_isBroken);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    m_EndGame.getInstance().setEndGameDriveSpeed(0.5);
    if((Timer.getFPGATimestamp() - m_initTime) > m_minimumTime){
      if(m_EndGame.getMexicoSensor()){
        m_EndGame.getInstance().setAnklesReleased(true);
        m_tripTime = Timer.getFPGATimestamp();
      }
      if((Timer.getFPGATimestamp() - m_tripTime) > m_timeAfterAnkles && m_EndGame.getAnklesReleased());
    }



  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    System.out.println("Finished AnklesBroken Command");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
