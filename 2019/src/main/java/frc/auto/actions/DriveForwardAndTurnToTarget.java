package frc.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import lib.util.DriveSignal;

import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAndTurnToTarget implements Action {
    private static final Drivetrain mDrive = Drivetrain.getInstance();
    private static final Limelight mLL = Limelight.getInstance();

    private double mStartTime;
    private final double mDuration;
    private boolean mFinished;

    private boolean validTarget;
    private double headingError;
    private double steeringAdjust;

    private double minimumCommand = Constants.DRIVE_FEEDFORWARD;
    private double testForwardPercent = 0.2;

    private double leftCommand = 0.0;
    private double rightCommand = 0.0;

    public DriveForwardAndTurnToTarget(double duration) {
        mDuration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration || mFinished; 
    }

    @Override
    public void update() {
        validTarget = mLL.IsTargeting();
        headingError = mLL.GetOffsetAngle();
        
        if (validTarget && headingError > 0.5) {
            steeringAdjust = Constants.kP_aim * headingError - minimumCommand;
        }
        else if(validTarget && headingError < 0.5) {
            steeringAdjust = Constants.kP_aim * headingError + minimumCommand;
        }
        else {
            mFinished = true;
        }

        leftCommand += steeringAdjust + testForwardPercent;
        rightCommand -= steeringAdjust + testForwardPercent;

        mDrive.setOpenLoop(new DriveSignal(leftCommand, rightCommand));

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }
}