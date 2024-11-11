package frc.robot.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConfig;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.*;

@Logged
public class Elevator extends Mechanism {

    public static class ElevatorConfig extends Config {
        /* Elevator constants in rotations */
        @Getter private double maxHeight = 29.8;
        @Getter private double minHeight = 0;

        /* Elevator positions in rotations */
        @Getter @Setter private double fullExtend = maxHeight;
        @Getter private double home = minHeight;
        @Getter private double amp = 15;
        @Getter private double trap = 5;

        @Getter private double ampTriggerHeightPct = 0.8;
        @Getter private double elevatorUpHeight = 5;

        /* Elevator config settings */
        @Getter private final double zeroSpeed = -0.2;
        @Getter private final double positionKp = 0.86; // 20 FOC // 10 Regular
        @Getter private final double positionKv = 0.13; // .12 FOC // .15 regular
        @Getter private double currentLimit = 20;
        @Getter private final double torqueCurrentLimit = 100;

        /* Sim properties */
        @Getter private double kElevatorGearing = 5;
        @Getter private double kCarriageMass = 1;
        @Getter private double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.955 / 2);
        @Getter private double initialX = 0.5;
        @Getter private double initialY = 0.0;
        @Getter private double angle = 180 - 72;
        @Getter private double staticLength = 20;
        @Getter private double movingLength = 20;

        public ElevatorConfig() {
            super("Elevator", 52, RobotConfig.CANIVORE);
            setFollowerConfigs(new FollowerConfig("left", 53, RobotConfig.CANIVORE, false));
            configPIDGains(0, positionKp, 0, 0);
            configFeedForwardGains(0, positionKv, 0, 0);
            configMotionMagic(700, 900, 0); // 40, 120 FOC // 120, 195 Regular
            configSupplyCurrentLimit(currentLimit, true);
            configStatorCurrentLimit(torqueCurrentLimit, true);
            configForwardTorqueCurrentLimit(torqueCurrentLimit);
            configReverseTorqueCurrentLimit(torqueCurrentLimit);
            configForwardSoftLimit(maxHeight, true);
            configReverseSoftLimit(minHeight, true);
            configNeutralBrakeMode(true);
            configCounterClockwise_Positive();
        }

        /** Use these method to set the config for the mechanism on each robot */
        public void configSupplyCurrentLimit(double currentLimit) {
            this.currentLimit = currentLimit;
            configSupplyCurrentLimit(currentLimit, true);
        }
    }

    private ElevatorConfig config;
    // private ElevatorSim sim;

    public Elevator(ElevatorConfig config) {
        super(config);
        this.config = config;

        // simulationInit();
        telemetryInit();
        RobotTelemetry.print(getName() + " Subsystem Initialized");
    }

    @Override
    public void periodic() {}

    /*-------------------
    initSendable
    Use # to denote items that are settable
    ------------*/
    @Override
    public void initSendable(NTSendableBuilder builder) {
        if (isAttached()) {
            builder.addDoubleProperty("Position", this::getMotorPosition, null);
            builder.addDoubleProperty("Velocity", this::getMotorVelocityRPM, null);
            builder.addDoubleProperty("#FullExtend", config::getFullExtend, config::setFullExtend);
        }
    }

    /* Check Elevator States */
    // Is Amp Height
    public Trigger isAtAmp() {
        return new Trigger(
                () -> (getMotorPosition() > config.getAmp() * config.getAmpTriggerHeightPct()));
    }

    public Trigger isUp() {
        return new Trigger(() -> (getMotorPosition() >= config.getElevatorUpHeight()));
    }

    // --------------------------------------------------------------------------------
    // Custom Commands
    // --------------------------------------------------------------------------------

    /** Holds the position of the elevator. */
    public Command holdPosition() {
        return new Command() {
            double holdPosition = 0; // rotations

            // constructor
            {
                setName("Elevator.holdPosition");
                addRequirements(Elevator.this);
            }

            @Override
            public void initialize() {
                stop();
                holdPosition = getMotorPosition();
            }

            @Override
            public void execute() {
                double currentPosition = getMotorPosition();
                if (Math.abs(holdPosition - currentPosition) <= 5) {
                    setMMPosition(() -> holdPosition);
                } else {
                    stop();
                    DriverStation.reportError(
                            "ElevatorHoldPosition tried to go too far away from current position. Current Position: "
                                    + currentPosition
                                    + " || Hold Position: "
                                    + holdPosition,
                            false);
                }
            }

            @Override
            public void end(boolean interrupted) {
                stop();
            }
        };
    }

    public Command zeroElevatorRoutine() {
        return new FunctionalCommand(
                        () -> toggleReverseSoftLimit(false), // init
                        () -> setPercentOutput(config::getZeroSpeed), // execute
                        (b) -> {
                            tareMotor();
                            toggleReverseSoftLimit(true); // end
                        },
                        () -> false, // isFinished
                        this) // requirement
                .withName("Elevator.zeroElevatorRoutine");
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    // private void simulationInit() {
    //     if (isAttached()) { // Only run simulation if it's attached
    //         sim = new ElevatorSim(motor.getSimState(), RobotSim.leftView);
    //     }
    // }

    // @Override
    // public void simulationPeriodic() {
    //     if (isAttached()) { // Only run if it's attached
    //         sim.simulationPeriodic();
    //     }
    // }

    // class ElevatorSim extends LinearSim {
    //     public ElevatorSim(TalonFXSimState elevatorMotorSim, Mechanism2d mech) {
    //         super(
    //                 new LinearConfig(
    //                                 config.initialX,
    //                                 config.initialY,
    //                                 config.kElevatorGearing,
    //                                 config.kElevatorDrumRadiusMeters)
    //                         .setAngle(config.angle)
    //                         .setMovingLength(config.getMovingLength())
    //                         .setStaticLength(config.getStaticLength()),
    //                 mech,
    //                 elevatorMotorSim,
    //                 config.getName());
    //     }
    // }
}
