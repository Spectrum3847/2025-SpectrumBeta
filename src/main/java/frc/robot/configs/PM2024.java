package frc.robot.configs;

public class PM2024 extends FM2024 {

    // We should be able to configure default LED colors per robot, so we know the right config is
    // loading

    public PM2024() {
        super();
        swerve.configEncoderOffsets(0.336426, -0.031006, -0.323730, 0.492188);
        launcher.setAttached(false);
        pivot.setAttached(false);
    }
}
