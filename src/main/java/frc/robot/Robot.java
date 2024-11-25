package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotConfig.ConfigHolder;
import frc.robot.amptrap.AmpTrap;
import frc.robot.auton.Auton;
import frc.robot.climber.Climber;
import frc.robot.elevator.Elevator;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.launcher.Launcher;
import frc.robot.leds.LedFull;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.pivot.Pivot;
import frc.robot.swerve.Swerve;
import frc.robot.vision.VisionSystem;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.util.CrashTracker;
import java.util.ArrayList;
import lombok.Getter;

public class Robot extends TimedRobot {
    @Getter private static RobotConfig robotConfig;
    @Getter private static ConfigHolder config;

    /** Create a single static instance of all of your subsystems */
    public static ArrayList<SpectrumSubsystem> subsystems = new ArrayList<SpectrumSubsystem>();

    @Getter private static RobotTelemetry telemetry;

    @Getter private static RobotSim robotSim;
    @Getter private static Swerve swerve;
    @Getter private static AmpTrap ampTrap;
    @Getter private static Climber climber;
    @Getter private static Elevator elevator;
    @Getter private static Feeder feeder;
    @Getter private static Intake intake;
    @Getter private static Launcher launcher;
    @Getter private static LedFull leds;
    @Getter private static Operator operator;
    @Getter private static Pilot pilot;
    @Getter private static Pivot pivot;
    @Getter private static VisionSystem visionSystem;
    @Getter private static Auton auton;

    public Robot() {
        DataLogManager.start();
        DriverStation.silenceJoystickConnectionWarning(true);
        try {
            RobotTelemetry.print("--- Robot Init Starting ---");
            robotSim = new RobotSim();

            /** Set up the config */
            robotConfig = new RobotConfig(); // Setup the robot config and choose which robot
            config = robotConfig.config; // This just makes it easier to access the config

            /**
             * Initialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are also subsystems.
             */
            double canInitDelay = 0.1; // Delay between any mechanism with motor/can configs

            leds = new LedFull(config.leds);
            operator = new Operator(config.operator);
            pilot = new Pilot(config.pilot);
            swerve = new Swerve(config.swerve);
            Timer.delay(canInitDelay);
            elevator = new Elevator(config.elevator);
            Timer.delay(canInitDelay);
            pivot = new Pivot(config.pivot);
            Timer.delay(canInitDelay);
            ampTrap = new AmpTrap(config.ampTrap);
            Timer.delay(canInitDelay);
            climber = new Climber(config.climber);
            Timer.delay(canInitDelay);
            feeder = new Feeder(config.feeder);
            Timer.delay(canInitDelay);
            intake = new Intake(config.intake);
            Timer.delay(canInitDelay);
            launcher = new Launcher(config.launcher);
            auton = new Auton();
            visionSystem = new VisionSystem(swerve::getRobotPose);

            /** Initialize Telemetry */
            telemetry = new RobotTelemetry();

            // Setup Default Commands for all subsystems
            subsystems.forEach(SpectrumSubsystem::setupDefaultCommand);

            RobotTelemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is initialized
     */
    public void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();

        // Bind Triggers for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupStates);
        RobotStates.setupStates();
    }

    public void clearCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Bind Triggers for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupStates);
        RobotStates.setupStates();
    }

    @Override // Deprecated
    public void robotInit() {}

    /** This method runs one time after the driver station connects. */
    @Override
    public void driverStationConnected() {}

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    @Override
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();

            SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        RobotTelemetry.print("### Disabled Init Starting ### ");
        resetCommandsAndButtons();

        RobotTelemetry.print("### Disabled Init Complete ### ");
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        RobotStates.coastMode.setFalse(); // Ensure motors are in brake mode
        RobotTelemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    @Override
    public void autonomousInit() {
        try {
            RobotTelemetry.print("@@@ Auton Init Starting @@@ ");
            clearCommandsAndButtons();

            auton.init();

            RobotTelemetry.print("@@@ Auton Init Complete @@@ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        auton.exit();
        RobotTelemetry.print("@@@ Auton Exit @@@ ");
    }

    @Override
    public void teleopInit() {
        try {
            RobotTelemetry.print("!!! Teleop Init Starting !!! ");
            resetCommandsAndButtons();

            RobotTelemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        RobotTelemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    @Override
    public void testInit() {
        try {

            RobotTelemetry.print("~~~ Test Init Starting ~~~ ");
            resetCommandsAndButtons();

            RobotTelemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {
        RobotTelemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    @Override
    public void simulationInit() {
        RobotTelemetry.print("$$$ Simulation Init Starting $$$ ");

        RobotTelemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    @Override
    public void simulationPeriodic() {}
}
