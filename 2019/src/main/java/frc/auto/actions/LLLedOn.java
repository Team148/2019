package frc.auto.actions;

// import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Limelight;

public class LLLedOn implements Action {
  public LLLedOn() {
  }

  // Called just before this Command runs the first time
  @Override
  public void start() {
    Limelight.getInstance().setLimelightLEDOn();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void update() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void done() {
  }

}
