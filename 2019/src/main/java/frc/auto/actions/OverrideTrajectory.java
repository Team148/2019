package frc.auto.actions;

import frc.robot.subsystems.Drivetrain;

public class OverrideTrajectory extends RunOnceAction {
    @Override
    public void runOnce() {
        Drivetrain.getInstance().overrideTrajectory(true);
    }
}
