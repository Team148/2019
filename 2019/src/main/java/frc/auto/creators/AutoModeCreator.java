package frc.auto.creators;

import frc.robot.AutoFieldState;
import frc.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}
