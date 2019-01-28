package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipModeEasy extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToCargoTwo;

    public CargoShipModeEasy(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        mLevel1ToCargoTwo = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToCargoTwo.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                mLevel1ToCargoTwo
            )
        ));

        // //Get Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(

        //     )
        // ));

        // //Score Second Hatch
        // runAction(new ParallelAction (
        //     Arrays.asList(
                
        //     )
        // ));
    }
}