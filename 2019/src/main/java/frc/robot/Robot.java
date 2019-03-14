/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;
import java.util.Optional; 

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;

//import Subsystems
import frc.robot.subsystems.*;
import frc.robot.subsystems.FloorBallIntake;
import frc.robot.subsystems.Beak;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndGame;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RollerClaw;
import frc.robot.SubsystemManager;

//import Commands
import frc.robot.commands.SetElevator;
import frc.robot.commands.SetElevatorShifter;
import frc.robot.commands.AutoHang;
import frc.robot.commands.SetEndGameHeight;
import frc.robot.commands.SetAnkle;
import frc.robot.commands.EndGameDrive;
import frc.robot.commands.HangStageFour;
import frc.robot.commands.HangStageOne;
import frc.robot.commands.HangStageThree;
import frc.robot.commands.HangStageTwo;
import frc.robot.commands.UpdateLimeLight;


//import 254
import frc.auto.AutoModeBase;
import frc.auto.AutoModeExecutor;
import frc.loops.Looper;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Pose2d;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import lib.util.*;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Looper mEnabledLooper = new Looper();
  private Looper mDisabledLooper = new Looper();

  private ArcadeDriveHelper mArcadeDriveHelper = new ArcadeDriveHelper();

  private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  private AutoModeExecutor mAutoModeExecutor;

  private final SubsystemManager mSubsystemManager = new SubsystemManager(
    Arrays.asList(
      RobotStateEstimator.getInstance(),
      Drivetrain.getInstance()
    )
  );

  public static FloorBallIntake m_Ball;
  public static Beak m_Beak;
  public static Drivetrain m_DriveTrain;
  public static Elevator m_Elevator;
  public static EndGame m_EndGame;
  public static Limelight m_Limelight;
  public static RollerClaw m_Claw;
  public static OI m_OI;

  private final Compressor comp = new Compressor(1);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() { 
    m_Ball = FloorBallIntake.getInstance();
    m_Beak = Beak.getInstance();
    m_DriveTrain = Drivetrain.getInstance();
    m_Elevator = Elevator.getInstance();
    m_EndGame = EndGame.getInstance();
    m_Limelight = Limelight.getInstance();
    m_Claw = RollerClaw.getInstance();
    m_OI = OI.getInstance();

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mSubsystemManager.registerDisabledLoops(mDisabledLooper);

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

    try {
      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();
      if (mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }

      Drivetrain.getInstance().zeroSensors();
      RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

      mAutoModeSelector.reset();
      mAutoModeSelector.updateModeCreator();
      mAutoModeExecutor = new AutoModeExecutor();

      mDisabledLooper.start();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();

    mAutoModeSelector.updateModeCreator();

    Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
    
    if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
      // System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
      mAutoModeExecutor.setAutoMode(autoMode.get());
    }
    System.gc();

    RobotState.getInstance().outputToSmartDashboard();
    Drivetrain.getInstance().outputTelemetry();
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
    
    try {
      comp.setClosedLoopControl(false);

      CrashTracker.logAutoInit();
      mDisabledLooper.stop();

      RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
      Drivetrain.getInstance().zeroSensors();

      mAutoModeExecutor.start();

      mEnabledLooper.start();
  } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
  }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

    RobotState.getInstance().outputToSmartDashboard();
    Drivetrain.getInstance().outputTelemetry();
  }

  @Override
  public void teleopInit() {
    try {
      CrashTracker.logTeleopInit();
      mDisabledLooper.stop();
      
      comp.setClosedLoopControl(true);

      if (mAutoModeExecutor != null) {
          mAutoModeExecutor.stop();
      }

      RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
      mEnabledLooper.start();

      m_DriveTrain.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
      m_DriveTrain.setOpenLoop(new DriveSignal(0.05, 0.05));

      if(!m_Elevator.isClosedLoop()) {
        m_Elevator.configClosedLoop();
      }

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // double timestamp = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("Match Time", timestamp);

    SmartDashboard.putString("Match Cycle", "TELEOP");
    SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getElevatorPosition());

    double throttle = m_OI.getThrottle();
    double turn = m_OI.getTurn();

    double ballIntakePercent = 0.0;
    double rollerClawPercent = 0.0;
    double feetPercent = 0.0;

    try {
        
        //driver inputs

        if(m_OI.m_driveJoystick.getRawButton(10)) {
          m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle * -1, turn));
        }
        else {
          m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle * -1, turn * 0.7));
        }
        

        //claw ball outtake (face buttons)
        if(m_OI.getDriverQuarterSpeed()) {
          rollerClawPercent = -0.25;
        }
        else if(m_OI.getDriverHalfSpeed()) {
          rollerClawPercent = -0.50;
        }
        else if(m_OI.getDriverThreeQuarterSpeed()) {
          rollerClawPercent = -0.75;
        }
        else if(m_OI.getDriverFullSpeed()) {
          rollerClawPercent = -1.0; 
        }

        //left bumper
        if(m_OI.getDriver5()) {
            m_Beak.setBeakGrab(true); //beak grab true means let go/open
            m_Beak.setBeakBar(true);
        }

        //right bumper
        if(m_OI.getDriver6()) {
            m_Beak.setBeakGrab(false);
        }

        //driver POV
        if(m_OI.m_driveJoystick.getPOV() == 270) {
          m_Beak.setBeakBar(false);
        }
        if(m_OI.m_driveJoystick.getPOV() == 90) {
          m_Beak.setBeakBar(true);
        }

        //driver and operator triggers
        if(m_OI.m_driveJoystick.getRawAxis(2) > 0.3) {
          rollerClawPercent = -1.0;
        }
        if(m_OI.m_driveJoystick.getRawAxis(3) > 0.3) {
          rollerClawPercent = 1.0;
        }

        if(m_OI.m_operatorJoystick.getRawAxis(2) > 0.3) {
          m_Ball.setBallIntakeCylinder(true);
          m_Beak.setBeakBar(true);
          m_Beak.setBeakGrab(false); //beak grab false means hang on/close
        }
        if(m_OI.m_operatorJoystick.getRawAxis(3) > 0.3) {
          m_Ball.setBallIntakeCylinder(false);
        }

        //operator inputs
        //face buttons
        if(m_OI.getEndgameSafety()) {
          if(m_OI.getEndgameManual()) {
            if(m_OI.getOperator1()) {
              Scheduler.getInstance().add(new HangStageOne());
            }

            // if(m_OI.getOperator2()) {
            //   Scheduler.getInstance().add(new HangStageTwo());
            // }
    
            if(m_OI.getOperator3()) {
              Scheduler.getInstance().add(new HangStageThree());
            }

            if(m_OI.getOperator4()) {
              Scheduler.getInstance().add(new HangStageFour());
            }
            if(m_OI.m_operatorJoystick.getRawAxis(5) < -0.2) {
              feetPercent = -1 * (Math.abs(m_OI.m_operatorJoystick.getRawAxis(5) * 0.5));
            }
          }
          else {
            if(m_OI.m_operatorJoystick.getRawButtonPressed(7) && m_OI.m_operatorJoystick.getRawButton(8)){
              Scheduler.getInstance().add(new AutoHang());
            }
          }
        }
        else {
          if(m_OI.getOperator1()) {
            m_Beak.setBeakBar(false);
          }
  
          if(m_OI.getOperator3()) {
            m_Beak.setBeakBar(true);
          }
        }
        

        //left bumper
        if(m_OI.getFloorIntake()) {
            ballIntakePercent = -1.0;
            rollerClawPercent = -0.6;
        }
        //right bumper
        if(m_OI.getFloorOuttake()) {
            ballIntakePercent = 1.0;
            rollerClawPercent = 1.0;
        }
        
        if(m_OI.m_operatorJoystick.getPOV() == 0) {
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_HIGH));
        }
        if(m_OI.m_operatorJoystick.getPOV() == 90) {
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_MIDDLE));
        }
        if(m_OI.m_operatorJoystick.getPOV() == 180) {
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_ZERO));
        }
        if(m_OI.m_operatorJoystick.getPOV() == 270) {
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_CARGO));
        }

        //set subsystems motors and soleno ids from inputs
        m_Ball.setBallIntakeMotor(ballIntakePercent);
        m_Claw.setRollerClaw(rollerClawPercent);
        m_EndGame.setEndGameDriveSpeed(feetPercent);

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}