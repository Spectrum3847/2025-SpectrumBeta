package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;

public class Operator extends Gamepad {

    // Triggers, these would be robot states such as ampReady, intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger intake_A = A.and(noFn, teleop);
    public final Trigger eject_fA = A.and(fn, teleop);

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    // TEST TRIGGERS

    public static class OperatorConfig extends Config {

        @Getter private final double triggersDeadzone = 0;

        public OperatorConfig() {
            super("Operator", 1);
        }
    }

    private OperatorConfig config;

    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;
        Robot.subsystems.add(this);
        RobotTelemetry.print("Operator Subsystem Initialized: ");
    }

    public void setupStates() {
        // Left Blank so we can bind when the controller is connected
        OperatorStates.setStates();
    }

    public void setupDefaultCommand() {
        OperatorStates.setupDefaultCommand();
    }
}
