package frc.auto.actions;

import frc.robot.subsystems.Drivetrain;
import lib.geometry.Rotation2d;

/**
 * Turns the robot to a specified heading
 * 
 * @see Action
 */
public class TurnToHeading implements Action {

    private Rotation2d mTargetHeading;
    private Drivetrain mDrive = Drivetrain.getInstance();

    public TurnToHeading(Rotation2d heading) {
        mTargetHeading = heading;
    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithTurn();
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        mDrive.setWantTurnToHeading(mTargetHeading);
    }
}