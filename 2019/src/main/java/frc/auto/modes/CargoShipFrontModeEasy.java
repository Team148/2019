package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Rotation2d;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipFrontModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToCargoOneLineupForward;

    public CargoShipFrontModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        if(mStartedLeft) {
            mLevel1ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoOneLineupForwardLeft, true, false, TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), true);
        
        }
        else {
            mLevel1ToCargoOneLineupForward = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().levelOneToCargoOneLineupForwardRight, true, false, TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(-4.0, -4.0)), TrajectoryGenerator.kCargoOneLineupPose.getTranslation().translateBy(new Translation2d(4.0, 4.0)), false);
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo AND Rocket Mode");

        //Score First Hatch
        runAction(new SeriesAction (
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new ExtendRetract4Bar(true),
                    mLevel1ToCargoOneLineupForward
                )),
                new DriveForwardAndTurnToTarget(20.0, 0.75),
                new OpenCloseBeak(true)
            )
        ));
        runAction(new SeriesAction (
            Arrays.asList(
                new OpenLoopDrive(-0.5, -0.5, 0.25),
                new TurnToHeading(Rotation2d.fromDegrees(240.0))
            )
        ));

        // //Score Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
                
        //     )
        // ));
    }
}