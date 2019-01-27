/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.sql.Driver;
import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//import Subsystems
import frc.robot.subsystems.BallFloorIntake;
import frc.robot.subsystems.Beak;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorBallRoller;
import frc.robot.subsystems.HatchFloorIntake;

//import Commands

//import 254
import frc.auto.AutoModeBase;
import frc.auto.AutoModeExecutor;
import frc.paths.TrajectoryGenerator;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static BallFloorIntake m_ballFloor;
  public static Beak m_beak;
  public static Drivetrain m_driveTrain;
  public static Elevator m_elevator;
  public static ElevatorBallRoller m_ballElevator;
  public static HatchFloorIntake m_hatchFloor;

  private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  private AutoModeExecutor mAutoModeExecutor;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    m_oi = OI.getInstance();
    m_ballFloor = BallFloorIntake.getInstance();
    m_beak = Beak.getInstance();
    m_driveTrain = Drivetrain.getInstance();
    m_elevator = Elevator.getInstance();
    m_ballElevator = ElevatorBallRoller.getInstance();
    m_hatchFloor = HatchFloorIntake.getInstance();

    


    mTrajectoryGenerator.generateTrajectories();
    mAutoModeSelector.updateModeCreator();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {

      // Reset all auto mode state.
      mAutoModeSelector.reset();
      mAutoModeSelector.updateModeCreator();
      mAutoModeExecutor = new AutoModeExecutor();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

    mAutoModeSelector.updateModeCreator();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    
    mAutoModeExecutor.start();
    Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
    mAutoModeExecutor.setAutoMode(autoMode.get());
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
  }

  @Override
  public void teleopInit() {
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putString("Match Cycle", "TELEOP");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

  }
}