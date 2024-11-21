package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.amptrap.AmpTrap.AmpTrapConfig;
import frc.robot.climber.Climber.ClimberConfig;
import frc.robot.configs.FM2024;
import frc.robot.configs.PM2024;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.feeder.Feeder.FeederConfig;
import frc.robot.intake.Intake.IntakeConfig;
import frc.robot.launcher.Launcher.LauncherConfig;
import frc.robot.leds.LedFull.LedFullConfig;
import frc.robot.operator.Operator.OperatorConfig;
import frc.robot.pilot.Pilot.PilotConfig;
import frc.robot.pivot.Pivot.PivotConfig;
import frc.robot.swerve.SwerveConfig;
import frc.spectrumLib.Elastic;
import frc.spectrumLib.Elastic.ElasticNotification;
import frc.spectrumLib.Elastic.ElasticNotification.NotificationLevel;

public class RobotConfig {

    public static class ConfigHolder {
        public SwerveConfig swerve;
        public AmpTrapConfig ampTrap;
        public ClimberConfig climber;
        public ElevatorConfig elevator;
        public FeederConfig feeder;
        public IntakeConfig intake;
        public LauncherConfig launcher;
        public LedFullConfig leds;
        public OperatorConfig operator;
        public PilotConfig pilot;
        public PivotConfig pivot;

        public ConfigHolder() {
            swerve = new SwerveConfig();
            ampTrap = new AmpTrapConfig();
            climber = new ClimberConfig();
            elevator = new ElevatorConfig();
            feeder = new FeederConfig();
            intake = new IntakeConfig();
            launcher = new LauncherConfig();
            leds = new LedFullConfig();
            operator = new OperatorConfig();
            pilot = new PilotConfig();
            pivot = new PivotConfig();
        }
    }

    public String rioSerial = "empty";
    public static final Double robotInitDelay = 2.0; // Seconds to wait before starting robot code

    public final String ALPHA2024SERIAL = "032B1F69";
    public final String PM2024SERIAL = "03223839";
    public final String ULTRAVIOLET2024SERIAL = "032B1F69"; // "0329AD07";
    public final String PHOTON2024SERIAL = "0329AD07";

    public static final String CANIVORE = "*"; // CANbus name is 3847
    public static final String RIO_CANBUS = "rio";

    public final int ledPWMport = 0;

    private RobotType robotType = null;

    public ConfigHolder config;

    // Add aditional robot types here, need to add them to the checkRobotType method and
    // the config switch statement
    public enum RobotType {
        AM,
        PM,
        FM,
        PHOTON,
        SIM
    }

    public RobotConfig() {

        if (Robot.isReal()) {
            Timer.delay(RobotConfig.robotInitDelay); // Wait for the robot to fully boot up
        }
        // Set the RoboRio Serial number for this robot, useful for adjusting comp/practice bot
        // settings
        if (RobotController.getSerialNumber() != null) {
            rioSerial = RobotController.getSerialNumber();
            System.out.println("RIO SERIAL: " + rioSerial);
        }

        checkRobotType();
        // Set Config based on which robot we are on
        switch (getRobotType()) {
            case AM:
                config = new ConfigHolder();
                break;
            case PM:
                config = new PM2024();
                break;
            case PHOTON:
                config = new ConfigHolder();
                break;
            case SIM: // SIM runs FM config, move to sim other configs if needed
            case FM:
            default:
                /* Set all the default configs */
                config = new FM2024();
                break;
        }

        RobotTelemetry.print("ROBOT: " + getRobotType());
    }

    /** Set the RobotType based on if simulation or the serial number of the RIO */
    public RobotType checkRobotType() {
        if (Robot.isSimulation()) {
            robotType = RobotType.SIM;
            RobotTelemetry.print("Robot Type: Simulation");

        } else if (rioSerial.equals(ULTRAVIOLET2024SERIAL)) {
            robotType = RobotType.FM;
            RobotTelemetry.print("Robot Type: FM 2024");
        } else if (rioSerial.equals(ALPHA2024SERIAL)) {
            robotType = RobotType.AM;
            RobotTelemetry.print("Robot Type: AM 2024");
        } else if (rioSerial.equals(PM2024SERIAL)) {
            robotType = RobotType.PM;
            RobotTelemetry.print("Robot Type: PM 2024");
        } else if (rioSerial.equals(PHOTON2024SERIAL)) {
            robotType = RobotType.PHOTON;
            RobotTelemetry.print("Robot Type: PHOTON 2024");
        } else {
            robotType = RobotType.FM;
            DriverStation.reportError(
                    "Could not match rio to robot config; defaulting to ULTRAVIOLET robot config",
                    false);
            RobotTelemetry.print("Robot Type: ULTRAVIOLET 2024");
        }

        Elastic.sendAlert(
                new ElasticNotification(
                        NotificationLevel.INFO, "Robot Type", robotType.name(), 10000));
        new Alert("RobotType: " + robotType.name(), AlertType.kInfo).set(true);
        return robotType;
    }

    public RobotType getRobotType() {
        return robotType;
    }
}
