package frc.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import lib.util.DriveSignal;

import static org.junit.jupiter.api.DynamicTest.stream;

import edu.wpi.first.wpilibj.Timer;

public class TurnToTarget implements Action {
    private static final Drivetrain mDrive = Drivetrain.getInstance();

    private double mStartTime;
    private final double mDuration;

    private double offsetAngle = Limelight.getInstance().GetOffsetAngle();
    private double headingError = -1 * offsetAngle;
    private double steeringAdjust;

    private double minimumCommand = Constants.DRIVE_FEEDFORWARD;

    private double leftCommand = 0.0;
    private double rightCommand = 0.0;

    public TurnToTarget(double duration) {
        mDuration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration;
    }

    @Override
    public void update() {
        System.out.println((Timer.getFPGATimestamp() - mStartTime) + " > " + mDuration);

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        if(offsetAngle > 1.0) {
            steeringAdjust = Constants.HEADING_P * headingError - minimumCommand;
        }
        else if(offsetAngle < 1.0) {
            steeringAdjust = Constants.HEADING_P * headingError + minimumCommand;
        }
        leftCommand += steeringAdjust;
        rightCommand -= steeringAdjust;

        mDrive.setOpenLoop(new DriveSignal(leftCommand, rightCommand));
        mStartTime = Timer.getFPGATimestamp();
    }
}