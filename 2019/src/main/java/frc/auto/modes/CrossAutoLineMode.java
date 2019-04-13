package frc.auto.modes;

import frc.auto.AutoModeBase;
import frc.auto.AutoModeEndedException;
import frc.auto.actions.CollectAccelerationData;
import frc.auto.actions.CollectVelocityData;
import frc.auto.actions.OpenLoopDrive;
import frc.auto.actions.WaitAction;
import frc.auto.actions.TurnToHeading;
import lib.geometry.Rotation2d;
import lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        // List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        // List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // System.out.println("Running Cross auto line");
        // runAction(new CollectVelocityData(velocityData, false, false));
        // runAction(new WaitAction(10.0));
        // runAction(new CollectAccelerationData(accelerationData, false, false));

        // DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        // System.out.println("ks: " + constants.ks);
        // System.out.println("kv: " + constants.kv);
        // System.out.println("ka: " + constants.ka);
        // runAction(new WaitAction(10.0));
        // runAction(new CollectAccelerationData(accelerationData, false, false));
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(0.5, 0.5, 2.0));
    }
}
