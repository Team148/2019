package frc.auto.creators;

import frc.auto.AutoModeBase;

public interface NewAutoModeCreator {
    AutoModeBase getStateDependentAutoMode();
}