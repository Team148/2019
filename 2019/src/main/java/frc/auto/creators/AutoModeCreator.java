package frc.auto.creators;

import frc.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode();
}