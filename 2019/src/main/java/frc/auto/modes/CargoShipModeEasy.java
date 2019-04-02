package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;
import frc.robot.subsystems.Limelight;

import java.util.Arrays;

import javax.swing.text.html.ParagraphView;

public class CargoShipModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private static final Limelight LL = Limelight.getInstance();
    private double angleOffset;
    private int inchOffset;

    final boolean mStartedLeft;

    private DriveTrajectory mLevel1ToCargoTwoLineupForward;

    private DriveTrajectory mCargoTwoAway;
    private DriveTrajectory mEndCargoTwoToLoadingStation;

    private DriveTrajectory mToLoadingStation;
    private DriveTrajectory mLoadingStationToCargoThreeLineup;

    public CargoShipModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForwardLeft, true);
        
            mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoLeft, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonLeft);

            mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupLeft, true);
        }
        else {
            mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForwardRight, true);
        
            mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwoRight, true, false);
            mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStatonRight);

            mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineupRight, true);
        }

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                new ExtendRetract4Bar(true),
                mLevel1ToCargoTwoLineupForward
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
            )
        ));

        runAction(new ParallelAction (
            Arrays.asList(
                mCargoTwoAway
                // new ExtendRetract4Bar(false)
            )
        ));

        // //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoTwoToLoadingStation,
                new DriveForwardAndTurnToTarget(20.0, 1.0),
                // new ExtendRetract4Bar(true),
                new OpenCloseBeak(false),
                new OpenLoopDrive(-0.15, -0.15, 0.5)
            )
        ));

        runAction(new SeriesAction (
            Arrays.asList(
                mLoadingStationToCargoThreeLineup,
                new DriveForwardAndTurnToTarget(40.0, 1.0)
                // new OpenCloseBeak(true),
                // new OpenLoopDrive(-0.2, -0.2, 0.5)
            )
        ));
    }
}