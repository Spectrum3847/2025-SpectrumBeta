package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.util.Elastic;
import frc.spectrumLib.util.Elastic.ElasticNotification;
import frc.spectrumLib.util.Elastic.ElasticNotification.NotificationLevel;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {

    // Triggers, these would be robot states such as ampReady, intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger intake_A = A.and(noFn, teleop);
    public final Trigger eject_fA = A.and(fn, teleop);
    public final Trigger ampPrep_B = B; // .and(noFn, teleop);
    public final Trigger score_RB = rightBumper.and(teleop);
    public final Trigger launchPrep_RT = rightTrigger.and(noFn, teleop);
    public final Trigger subwooferPrep_fRT = rightTrigger.and(fn, teleop);
    public final Trigger climbPrep_RDP = rightDpad.and(noFn, teleop);
    public final Trigger climbRoutine_start = start.and(noFn, teleop);

    public final Trigger retract_X = X.and(noFn, teleop);
    public final Trigger manual_Y = Y.and(noFn, teleop);

    // Drive Triggers
    public final Trigger upReorient = upDpad.and(fn, teleop);
    public final Trigger leftReorient = leftDpad.and(fn, teleop);
    public final Trigger downReorient = downDpad.and(fn, teleop);
    public final Trigger rightReorient = rightDpad.and(fn, teleop);

    /* Use the right stick to set a cardinal direction to aim at */
    public final Trigger driving;
    public final Trigger steer;

    public final Trigger snapSteer = Trigger.kFalse;

    public final Trigger fpv_rs = rightStick.and(teleop); // Remapped to Right back button

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    // TEST TRIGGERS
    public final Trigger tuneElevator_tB = testMode.and(B);

    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.75;
        @Getter @Setter private double turboModeScalor = 1;
        private double deadzone = 0.001;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setRightStickDeadzone(deadzone);
            setRightStickExp(2.0);
            setRightStickScalor(12);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalor(1);
        }
    }

    private PilotConfig config;

    @Getter @Setter
    private boolean isSlowMode = false; // TODO: change slow and turbo to SpectrumStates

    @Getter @Setter private boolean isTurboMode = false;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;
        Robot.subsystems.add(this);

        driving =
                leftXTrigger(Threshold.ABS_GREATER, config.deadzone)
                        .or(leftYTrigger(Threshold.ABS_GREATER, config.deadzone));
        steer =
                rightXTrigger(Threshold.ABS_GREATER, config.deadzone)
                        .or(rightYTrigger(Threshold.ABS_GREATER, config.deadzone));

        RobotTelemetry.print("Pilot Subsystem Initialized: ");
        Elastic.sendAlert(
                new ElasticNotification(NotificationLevel.INFO, "Pilot Subsystem Initialized", ""));
    }

    public void setupStates() {
        // Used for setting rumble and control mode states only
        PilotStates.setStates();
    }

    public void setupDefaultCommand() {
        PilotStates.setupDefaultCommand();
    }

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return -1 * ccwPositive; // invert the value
    }

    // ELEVATOR METHODS
    public double getElevatorManualAxis() {
        return getLeftY();
    }
}
