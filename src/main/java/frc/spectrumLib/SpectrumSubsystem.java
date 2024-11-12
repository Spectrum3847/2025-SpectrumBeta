package frc.spectrumLib;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SpectrumSubsystem extends Subsystem {

    void bindTriggers();

    void setupDefaultCommand();
}