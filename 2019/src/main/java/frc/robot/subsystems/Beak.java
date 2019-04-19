package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Beak extends Subsystem {

  private static Beak m_instance;

  private final Solenoid m_beakBar = new Solenoid(RobotMap.BEAK_4BAR_SOLENOID);
  private final Solenoid m_beakGrab = new Solenoid(RobotMap.BEAK_GRAB_SOLENOID);

  public Beak() {

    super();

  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new MySpecialCommand());
  }

  public static Beak getInstance() { 
    if (m_instance == null) {
      m_instance = new Beak();
    }
    return m_instance;
  }

  public void setBeakBar (boolean on) {
    if (on) {
      m_beakBar.set(true);
    }
    else {
      m_beakBar.set(false);
    }
  }

  public void setBeakGrab (boolean on) {
    // System.out.println("Settings Beak Fingers!!!!!!!!!!!!!");
    if (on) {
      m_beakGrab.set(true);
      // System.out.println("!!!!Beak set to On");
    }
    else {
      m_beakGrab.set(false);
    }
  }
}
