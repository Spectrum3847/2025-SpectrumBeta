package frc.robot.amptrap;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.amptrap.AmpTrap.AmpTrapConfig;
import frc.spectrumLib.util.Elastic;
import frc.spectrumLib.util.Elastic.ElasticNotification;
import frc.spectrumLib.util.Elastic.ElasticNotification.NotificationLevel;

public class AmpTrapStates {
    private static AmpTrap ampTrap = Robot.getAmpTrap();
    private static AmpTrapConfig config = Robot.getConfig().ampTrap;

    // TODO: implement amptrap states

    public static void setupDefaultCommand() {
        ampTrap.setDefaultCommand(
                ampTrap.runStop().ignoringDisable(true).withName("AmpTrap.default"));
    }

    public static void setStates() {
        intaking.whileTrue(intake());
    }

    public static Command runFull() {
        return ampTrap.runVelocity(config::getMaxSpeed).withName("AmpTrap.runFull");
    }

    public static Command feed() {
        return ampTrap.runVelocity(config::getFeed).withName("AmpTrap.feed");
    }

    public static Command intake() {
        return ampTrap.runVelocity(config::getIntake)
                .withName("AmpTrap.intake")
                .alongWith(
                        Commands.runOnce(
                                () ->
                                        Elastic.sendAlert(
                                                new ElasticNotification(
                                                        NotificationLevel.INFO,
                                                        "Amptrap intaking",
                                                        "",
                                                        3000))));
    }

    public static Command amp() {
        return ampTrap.runVelocity(config::getAmp).withName("AmpTrap.amp");
    }

    public static Command score() {
        return ampTrap.runVelocity(config::getScore).withName("AmpTrap.score");
    }

    public static Command eject() {
        return ampTrap.runVelocity(config::getEject).withName("AmpTrap.eject");
    }

    public static Command stopMotor() {
        return ampTrap.runStop().withName("AmpTrap.stopMotor");
    }

    public static Command coastMode() {
        return ampTrap.coastMode();
    }

    /** Sets amptrap to coast mode. Does not automatically set it back to brake mode */
    public static Command stayCoastMode() {
        return ampTrap.stayCoastMode();
    }

    public static Command ensureBrakeMode() {
        return ampTrap.ensureBrakeMode();
    }
}
