package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class CargoShipANDRocketModeHard extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    
    final boolean mStartedLeft;

    public CargoShipANDRocketModeHard(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cargo AND Rocket Mode");

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