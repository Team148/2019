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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//import Subsystems
import frc.robot.subsystems.*;
import frc.robot.subsystems.FloorBallIntake;
import frc.robot.subsystems.FloorDiscIntake;
import frc.robot.subsystems.Beak;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.RollerClaw;

import frc.robot.SubsystemManager;

//import Commands
import frc.robot.commands.SetElevator;

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
  public static OI m_OI;
  public static FloorBallIntake m_Ball;
  public static Beak m_Beak;
  public static FloorDiscIntake m_Disc;
  public static Drivetrain m_DriveTrain;
  public static Elevator m_Elevator;
  public static RollerClaw m_Claw;

  private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  private AutoModeExecutor mAutoModeExecutor;

  private Looper mEnabledLooper = new Looper();
  private Looper mDisabledLooper = new Looper();

  private ArcadeDriveHelper mArcadeDriveHelper = new ArcadeDriveHelper();

  private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drivetrain.getInstance()
            )
    );

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    
    m_OI = OI.getInstance();
    m_Ball = FloorBallIntake.getInstance();
    m_Beak = Beak.getInstance();
    m_Disc = FloorDiscIntake.getInstance();
    m_DriveTrain = Drivetrain.getInstance();
    m_Elevator = Elevator.getInstance();
    m_Claw = RollerClaw.getInstance();

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

        mEnabledLooper.start();

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
      System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
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
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());

    RobotState.getInstance().outputToSmartDashboard();
    Drivetrain.getInstance().outputTelemetry();
  }

  @Override
  public void teleopInit() {
    try {
      CrashTracker.logTeleopInit();
      mDisabledLooper.stop();
      if (mAutoModeExecutor != null) {
          mAutoModeExecutor.stop();
      }

      // mInfrastructure.setIsDuringAuto(false);
      // mWrist.setRampRate(Constants.kWristRampRate);

      RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
      mEnabledLooper.start();
      // mLED.setEnableFaults(false);
      // mInHangMode = false;
      // mForklift.retract();

      // mShootDelayed.update(false, Double.POSITIVE_INFINITY);
      // mPoopyShootDelayed.update(false, Double.POSITIVE_INFINITY);
      m_DriveTrain.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
      m_DriveTrain.setOpenLoop(new DriveSignal(0.05, 0.05));

      // mKickStandEngaged = true;
      // mKickStandReleased.update(true);
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

    SmartDashboard.putString("Match Cycle", "TELEOP");
    SmartDashboard.putNumer("Time", Timer.getFPGATimestamp());
        // double timestamp = Timer.getFPGATimestamp();

        boolean ballMode = m_OI.getBallMode();
        boolean discWithSensor = m_OI.getDiscGrabWithSensor();
        boolean endGameSafety = m_OI.getEndgameSafety();
        boolean FPGAEndgame = m_OI.getFPGAEndgame();

        double throttle = m_OI.getThrottle();
        double turn = m_OI.getTurn();

        boolean beakGrab = false;
        boolean beak4Bar = false;
        boolean ballIntake = false;
        boolean discIntake = false;
        boolean endGame = false;

        double ballIntakePercent = 0.0;
        double discIntakePercent = 0.0;
        double rollerClawPercent = 0.0;

        try {
            
            //driver inputs
            m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle, turn));

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
              if(ballMode) {
                rollerClawPercent = -0.8;
              }
              else {
                beakGrab = true;
              }
            }

            //right bumper
            if(m_OI.getDriver6()) {
              if(ballMode) {
                rollerClawPercent = 0.8;
              }
              else {
                beakGrab = false;
              }
            }

            //operator inputs
            //face buttons
            if(m_OI.getOperator4BarIn() || (m_OI.m_driveJoystick.getPOV() == 270)) {
              beak4Bar = false;
            }

            if(m_OI.getOperator4BarOut() || (m_OI.m_driveJoystick.getPOV() == 90)) {
              beak4Bar = true;
            }

            if(m_OI.getOperatorDiscIntakeUp()) {
              discIntake = false;
            }

            //left bumper
            if(m_OI.getFloorIntake()) {
              if(ballMode) {
                ballIntakePercent = 0.5;
              }
              else {
                discIntakePercent = 0.5;
              }
            }

            //right bumper
            if(m_OI.getFloorOuttake()) {
              if(ballMode) {
                ballIntakePercent = -0.5;
              }
              else {
                discIntakePercent = -0.5;
              }
            }

            //elevator presets w/ dPad
            // if(m_OI.m_operatorJoystick.getPOV() == 0) {
            //   Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_HIGH));
            // }
            // if(m_OI.m_operatorJoystick.getPOV() == 90) {
            //   Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_MIDDLE));
            // }
            // if(m_OI.m_operatorJoystick.getPOV() == 180) {
            //   Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_ZERO));
            // }

            //deploy endgame
            //ADD FPGA checks for an auto-deploy
            if(!endGameSafety && (m_OI.getDriverEndgame() || m_OI.getOperatorEndgame())) {
              endGame = true;
            }

            //set subsystems motors and solenoids from inputs
            m_Ball.setBallIntakeCylinder(ballIntake);
            m_Ball.setBallIntakeMotor(ballIntakePercent);
            m_Beak.setBeakGrab(beakGrab);
            m_Beak.setBeakIn(beak4Bar);
            m_Claw.setRollerClaw(rollerClawPercent);
            m_Disc.setDiscIntakeCylinder(discIntake);
            m_Disc.setDiscIntakeMotor(discIntakePercent);

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