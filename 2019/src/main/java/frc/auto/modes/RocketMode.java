package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class RocketMode extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    final boolean mStartedLevelOne;
    final boolean mStartedLeft;

    public RocketMode(boolean driveOffLevel1, boolean driveToLeftCargo) {
        mStartedLevelOne = driveOffLevel1;
        mStartedLeft = driveToLeftCargo;

        if (driveOffLevel1 && driveToLeftCargo) {   //Level 1 AND Left

        }
        else if (driveOffLevel1 && !driveToLeftCargo) { //Level 1 AND Right

        }
        else if (!driveOffLevel1 && driveToLeftCargo) { //Level 2 AND Left

        }
        else {  //!driveOffLevel1 && !driveToLeftCargo  Level 2 AND Right

        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(

            )
        ));

        //Get Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(

            )
        ));

        //Score Second Hatch
        runAction(new ParallelAction (
            Arrays.asList(
                
            )
        ));
    }
}