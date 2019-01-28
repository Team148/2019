package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;
import lib.geometry.Translation2d;

import java.util.Arrays;

public class RocketModeHard extends AutoModeBase {

    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    final boolean mStartedLeft;
    private DriveTrajectory mLevel1ToRocketThree;

    public RocketModeHard(boolean driveToLeftCargo) {
        mStartedLeft = driveToLeftCargo;

        mLevel1ToRocketThree = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().level1ToRocketThree.get(mStartedLeft));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Rocket Mode");

        //Score First Hatch
        runAction(new ParallelAction (
            Arrays.asList(

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