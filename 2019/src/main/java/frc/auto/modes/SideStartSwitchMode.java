package frc.auto.modes;

import frc.auto.AutoConstants;
import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.*;
import frc.paths.TrajectoryGenerator;

import java.util.Arrays;

public class SideStartSwitchMode extends AutoModeBase {

    final boolean mGoLeft;
    final boolean mStartedLeft;
    private DriveTrajectory mTrajectory;

    public SideStartSwitchMode(boolean robotStartedOnLeft, boolean switchIsLeft) {
        mStartedLeft = robotStartedOnLeft;
        mGoLeft = switchIsLeft;

        if (mGoLeft == mStartedLeft) {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearSwitch.get(mStartedLeft), true);
        } else {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToFarSwitch.get(mStartedLeft), true);
        }
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");

        runAction(new ParallelAction(
                Arrays.asList(
                        mTrajectory
                )
        ));
    }
}
