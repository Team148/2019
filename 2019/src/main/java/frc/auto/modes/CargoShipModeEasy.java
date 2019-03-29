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

    public CargoShipModeEasy(boolean driveToLeftRocket) {
        mStartedLeft = driveToLeftRocket;

        mLevel1ToCargoTwoLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoTwoLineupForward.get(mStartedLeft), true);
        
        mCargoTwoAway = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().awayFromCargoTwo.get(mStartedLeft), true, false);
        mEndCargoTwoToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().endCargoTwoToLoadingStaton.get(mStartedLeft));

        // mToLoadingStation = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().cargoTwoLineupToLoadingStation.get(mStartedLeft));

        // mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineup.get(mStartedLeft), true, false);
        mLoadingStationToCargoThreeLineup = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().loadingStationToCargoThreeLineup.get(mStartedLeft), true);

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
                mCargoTwoAway,
                new ExtendRetract4Bar(false)
            )
        ));

        // //Get Second Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                mEndCargoTwoToLoadingStation,
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new ExtendRetract4Bar(true),
                new OpenLoopDrive(-0.1, -0.1, 0.2),
                new OpenCloseBeak(false)
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