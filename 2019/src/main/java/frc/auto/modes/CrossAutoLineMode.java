package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.OpenLoopDrive;
import frc.auto.actions.WaitAction;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(new WaitAction(5.0));
        // runAction(new OpenLoopDrive(-0.3, -0.3, 5.0, false));
        runAction(new OpenLoopDrive(-1.0, -1.0, 3.0, false));
    }
}
