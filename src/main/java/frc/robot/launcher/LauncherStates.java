package frc.robot.launcher;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.launcher.Launcher.LauncherConfig;
import java.util.function.DoubleSupplier;

public class LauncherStates {
    private static Launcher launcher = Robot.getLauncher();
    private static LauncherConfig config = Robot.getConfig().launcher;

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static void setStates() {
        subwooferPrep.whileTrue(subwooferRPM());
        speakerPrep.whileTrue(defaultLauncherRPM());
        ejecting.whileTrue(ejectRPM());

        coastMode.onTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
    }

    private static Command defaultLauncherRPM() {
        return runVelocity(config::getDefaultLaunchRPM).withName("Launcher.defaultLauncherRPM");
    }

    private static Command subwooferRPM() {
        return runVelocity(config::getSubwooferRPM).withName("Launcher.subwooferRPM");
    }

    private static Command ejectRPM() {
        return runVelocity(config::getEjectRPM).withName("Launcher.ejectRPM");
    }

    public static Command distanceVelocity(DoubleSupplier distanceMeter) {
        return runVelocity(() -> launcher.getRPMfromDistance(distanceMeter))
                .withName("Launcher.distanceVelocity");
    }

    private static Command runVelocity(DoubleSupplier velocityRPM) {
        return launcher.runVelocityTcFocRpm(velocityRPM).withName("Launcher.runVelocity");
    }

    private static Command coastMode() {
        return launcher.coastMode().withName("Launcher.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return launcher.ensureBrakeMode().withName("Launcher.BrakeMode");
    }
}
