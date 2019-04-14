package frc.auto.actions;

import frc.robot.subsystems.Limelight;

public class LLLedBlink implements Action {
  public LLLedBlink() {
  }

  // Called just before this Command runs the first time
  @Override
  public void start() {
    Limelight.getInstance().setLimelightLEDBlink();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void update() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void done() {
  }

}
