/*Team 148 - The Robowranglers, wish to extend a thank you to 
/ Team 254 - The Cheesy Poofs 
/ for their distribution of the source code that Overhang's source code is based on.

/ The original 254 source code is located here: https://github.com/Team254/FRC-2018-Public
/ License from 254 2018 Repository
MIT License

Copyright (c) 2018 Team 254

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
package frc.robot;

import java.util.Arrays;
import java.util.Optional; 

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//import Subsystems
import frc.robot.subsystems.*;
import frc.robot.subsystems.FloorBallIntake;
import frc.robot.subsystems.Beak;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndGame;
import frc.robot.subsystems.Forks;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RollerClaw;
import frc.robot.SubsystemManager;

//import Commands
import frc.robot.commands.SetElevator;
import frc.robot.commands.AutoHang;
import frc.robot.commands.HangStageFour;
import frc.robot.commands.HangStageOne;
import frc.robot.commands.HangStageThree;
import frc.robot.commands.HangStageTwo;


//import 254
import frc.auto.AutoModeBase;
import frc.auto.AutoModeExecutor;
import frc.loops.Looper;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Pose2d;
import lib.util.*;

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
  public static Forks m_Forks;
  public static Limelight m_Limelight;
  public static RollerClaw m_Claw;
  public static OI m_OI;

  private final Compressor comp = new Compressor(1);

  private boolean autoInterrupted = false;

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
    m_Forks = Forks.getInstance();
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
      System.out.println("IN DISABLED INIT");
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
    // System.out.println("IN DISABLED PERIODIC");
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
    SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getElevatorPosition());
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
      System.out.println("IN AUTONOMOUS INIT");
      CrashTracker.logAutoInit();
      mDisabledLooper.stop();
      
      m_Limelight.SetEnableVision(true);
      m_Limelight.setLimelightPipeline(0);
      autoInterrupted = false;
      comp.setClosedLoopControl(false);

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

    // SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

    if (m_OI.m_driveJoystick.getRawButton(7) && m_OI.m_driveJoystick.getRawButton(8) && !autoInterrupted) {
      autoInterrupted = true;
      disabledInit();
      disabledPeriodic();
      teleopInit();

    }

    if (autoInterrupted) {
      teleopPeriodic();
    }

    RobotState.getInstance().outputToSmartDashboard();
    Drivetrain.getInstance().outputTelemetry();
  }

  @Override
  public void teleopInit() {
    try {
      System.out.println("IN TELEOP INIT");
      CrashTracker.logTeleopInit();
      mDisabledLooper.stop();

      m_Limelight.SetEnableVision(false);
      m_Limelight.setLimelightPipeline(3);
      
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

    double throttle = m_OI.getThrottle();
    double turn = m_OI.getTurn();

    double ballIntakePercent = 0.0;
    double rollerClawPercent = 0.0;
    double feetPercent = 0.0;

    try {
        
        //driver inputs
        if(m_OI.getDriverVision()){
          m_Limelight.SetEnableVision(true);
          m_Limelight.setLimelightPipeline(0);
          if(m_Limelight.IsTargeting()){
            double limeX = m_Limelight.GetOffsetAngle();
            double cameraSteer = 0;

            double kCameraDrive = Constants.kCameraDriveClose;

            if (Math.abs(limeX) <= Constants.kCameraClose) {

                kCameraDrive = Constants.kCameraDriveClose;

            } else if (Math.abs(limeX) < Constants.kCameraMid) {

                kCameraDrive = Constants.kCameraDriveMid;

            } else if (Math.abs(limeX) < Constants.kCameraFar) {

                kCameraDrive = Constants.kCameraDriveFar;

            }

            cameraSteer = limeX * kCameraDrive; //-0.11 for POS Gold
 
            System.out.println("CameraSteer: " + cameraSteer);
        
            m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle * -1, cameraSteer)); 
            }
          else{

            m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle * -1, turn)); 
          }
            
        }

        //driver inputs - default case manual driving
        else{
          m_Limelight.SetEnableVision(false);
          m_Limelight.setLimelightPipeline(3);
          m_DriveTrain.setOpenLoop(mArcadeDriveHelper.arcadeDrive(throttle * -1, turn));
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

            if(m_OI.getOperator2()) {
              Scheduler.getInstance().add(new HangStageTwo());
            }
    
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
        if(m_OI.m_operatorJoystick.getPOV() == 270){
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_CARGO));
        }

        if(m_OI.getDriver4() && (m_Elevator.getElevatorMPosition() == 0)) {
          m_Beak.setBeakGrab(false);
          Scheduler.getInstance().add(new SetElevator(Constants.ELEVATOR_LOW_GOAL));
        }

        //set subsystems motors and solenoids from inputs
        m_Ball.setBallIntakeMotor(ballIntakePercent);
        m_Claw.setRollerClaw(rollerClawPercent);
        m_EndGame.setEndGameDriveSpeed(feetPercent);

        RobotState.getInstance().outputToSmartDashboard();
        Drivetrain.getInstance().outputTelemetry();
        SmartDashboard.putNumber("Elevator Encoder", m_Elevator.getElevatorPosition());

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