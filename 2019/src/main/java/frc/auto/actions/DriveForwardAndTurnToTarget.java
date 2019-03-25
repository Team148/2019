package frc.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

import lib.util.DriveSignal;
import lib.util.SynchronousPIDF;

import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAndTurnToTarget implements Action {
    private static final Drivetrain mDrive = Drivetrain.getInstance();
    private static final Limelight mLL = Limelight.getInstance();

    private static final SynchronousPIDF mPID = new SynchronousPIDF(Constants.LL_K_P, Constants.LL_K_I, Constants.LL_K_D);

    private double mStartTime;
    private final double mDuration;
    private boolean mFinished;
    private double mVelocity; 

    private boolean validTarget;
    private double headingError;
    private double steeringAdjust;
    private double lastTime;
    private double maxsteeringAdjust = 0.175;

    // private double minimumCommand = Constants.DRIVE_FEEDFORWARD;

    private double leftCommand = 0.0;
    private double rightCommand = 0.0;

    public DriveForwardAndTurnToTarget(double velocity, double duration) {
        mVelocity = velocity;
        mDuration = duration;
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration || mFinished; 
    }

    @Override
    public void update() {

        

        validTarget = mLL.IsTargeting();
        headingError = mLL.GetOffsetAngle();

        double dt = Timer.getFPGATimestamp() - lastTime;

        steeringAdjust = -mPID.calculate(headingError, dt);
        
        // if (validTarget && headingError > 0.5) {
        //     steeringAdjust = Constants.kP_aim * headingError - minimumCommand;
        // }
        // else if(validTarget && headingError < 0.5) {
        //     steeringAdjust = Constants.kP_aim * headingError + minimumCommand;
        // }
        // else {
        //     mFinished = true;
        // }

        if(steeringAdjust > maxsteeringAdjust){
            steeringAdjust = maxsteeringAdjust;
            System.out.println("Max Steering Adjust");
        }
        if(steeringAdjust < -maxsteeringAdjust){
        steeringAdjust = -maxsteeringAdjust;
        System.out.println("Max Steering Adjust");
        }

        System.out.println("Steering Adjust: " + steeringAdjust);


        leftCommand =  mVelocity + steeringAdjust;
        rightCommand  = mVelocity - steeringAdjust;

        double highest_vel = Math.max(Math.abs(leftCommand), Math.abs(rightCommand));

        if(highest_vel > 1.0){
            leftCommand /= highest_vel;
            rightCommand /= highest_vel;
        }


        mDrive.setOpenLoop(new DriveSignal(leftCommand, rightCommand));

        lastTime = Timer.getFPGATimestamp();

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
        mLL.SetFastNT(false);
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mLL.SetFastNT(true);
    }
}