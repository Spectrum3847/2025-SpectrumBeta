package frc.spectrumLib.vision;

import frc.robot.vision.VisionSystem.Pose2dSupplier;
import frc.spectrumLib.SpectrumSubsystem;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSystem implements SpectrumSubsystem {

    private final VisionSystemSim visionSim = new VisionSystemSim("VisionSim");

    public static class Config {
        private Pose2dSupplier getSimPose; // Method that returns the robot Post
    }

    @Override
    public void setupStates() {}

    @Override
    public void setupDefaultCommand() {}
}
