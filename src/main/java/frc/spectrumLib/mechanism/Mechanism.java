package frc.spectrumLib.mechanism;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.spectrumLib.talonFX.TalonFXFactory;
import frc.spectrumLib.util.CanDeviceId;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.*;

/**
 * Control Modes Docs:
 * https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html
 * Closed-loop & Motion Magic Docs:
 * https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public abstract class Mechanism implements Subsystem, NTSendable {
    @Getter protected TalonFX motor;
    @Getter protected TalonFX[] followerMotors;
    public Config config;

    public Mechanism(Config config) {
        this.config = config;

        if (isAttached()) {
            motor = TalonFXFactory.createConfigTalon(config.id, config.talonConfig);

            followerMotors = new TalonFX[config.followerConfigs.length];
            for (int i = 0; i < config.followerConfigs.length; i++) {
                followerMotors[i] =
                        TalonFXFactory.createPermanentFollowerTalon(
                                config.followerConfigs[i].id,
                                motor,
                                config.followerConfigs[i].opposeLeader);
            }
        }
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public Mechanism(Config config, boolean attached) {
        this(config);
        config.attached = attached;
    }

    // Setup the telemetry values, has to be called at the end of the implemetned mechanism
    // constructor
    public void telemetryInit() {
        SendableRegistry.add(this, getName());
        SmartDashboard.putData(this);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public String getName() {
        return config.getName();
    }

    public boolean isAttached() {
        return config.isAttached();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType(getName());
    }

    /**
     * Gets the position of the motor
     *
     * @return motor position in rotations
     */
    @Logged
    public double getMotorPosition() {
        if (config.attached) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0;
    }

    /**
     * Gets the velocity of the motor
     *
     * @return motor velocity in rotations/sec which are the CTRE native units
     */
    public double getMotorVelocityRPS() {
        if (config.attached) {
            return motor.getVelocity().getValueAsDouble();
        }
        return 0;
    }

    // Get Velocity in RPM
    @Logged
    public double getMotorVelocityRPM() {
        return Conversions.RPStoRPM(getMotorVelocityRPS());
    }

    /* Commands: see method in lambda for more information */
    /**
     * Runs the Mechanism at a given velocity
     *
     * @param velocity in revolutions per minute
     */
    public Command runVelocity(DoubleSupplier velocity) {
        return run(() -> setVelocity(() -> Conversions.RPMtoRPS(velocity)))
                .withName(getName() + ".runVelocity");
    }

    /**
     * Run the mechanism at given velocity rpm in TorqueCurrentFOC mode
     *
     * @param velocityRPM
     * @return
     */
    public Command runVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        return run(() -> setVelocityTorqueCurrentFOC(() -> Conversions.RPMtoRPS(velocityRPM)))
                .withName(getName() + ".runVelocityFOCrpm");
    }

    public Command runPercentage(DoubleSupplier percentSupplier) {
        return run(() -> setPercentOutput(percentSupplier)).withName(getName() + ".runPercentage");
    }

    /**
     * Run to the specified position.
     *
     * @param position position in revolutions
     */
    public Command moveToPoseRevolutions(DoubleSupplier position) {
        return run(() -> setMMPosition(position)).withName(getName() + ".runPoseRevolutions");
    }

    /**
     * Move to the specified position.
     *
     * @param position position in percentage of max revolutions
     */
    public Command moveToPosePercentage(DoubleSupplier position) {
        return run(() -> setMMPosition(() -> config.maxRotation * (position.getAsDouble() / 100)))
                .withName(getName() + ".runPosePercentage");
    }

    /**
     * Runs to the specified position using FOC control. Will require different PID and feedforward
     * configs
     *
     * @param position position in revolutions
     */
    public Command runFOCPosition(DoubleSupplier position) {
        return run(() -> setMMPositionFOC(position)).withName(getName() + ".runFOCPosition");
    }

    public Command runStop() {
        return run(() -> stop()).withName(getName() + ".runStop");
    }

    /**
     * Temporarily sets the mechanism to coast mode. The configuration is applied when the command
     * is started and reverted when the command is ended.
     */
    public Command coastMode() {
        return startEnd(() -> setBrakeMode(false), () -> setBrakeMode(true))
                .ignoringDisable(true)
                .withName(getName() + ".coastMode");
    }

    /** Sets the motor to brake mode if it is in coast mode */
    public Command ensureBrakeMode() {
        return runOnce(
                        () -> {
                            setBrakeMode(true);
                        })
                .onlyIf(
                        () ->
                                config.attached
                                        && config.talonConfig.MotorOutput.NeutralMode
                                                == NeutralModeValue.Coast)
                .ignoringDisable(true)
                .withName(getName() + ".ensureBrakeMode");
    }

    protected void stop() {
        if (isAttached()) {
            motor.stopMotor();
        }
    }

    /** Sets the mechanism position of the motor to 0 */
    protected void tareMotor() {
        if (isAttached()) {
            setMotorPosition(() -> 0);
        }
    }

    /**
     * Sets the mechanism position of the motor
     *
     * @param position rotations
     */
    protected void setMotorPosition(DoubleSupplier position) {
        if (isAttached()) {
            motor.setPosition(position.getAsDouble());
        }
    }

    /**
     * Closed-loop Velocity Motion Magic with torque control (requires Pro)
     *
     * @param velocity rotations per second
     */
    protected void setMMVelocityFOC(DoubleSupplier velocity) {
        if (isAttached()) {
            MotionMagicVelocityTorqueCurrentFOC mm =
                    config.mmVelocityFOC.withVelocity(velocity.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocity rotations per second
     */
    protected void setVelocityTorqueCurrentFOC(DoubleSupplier velocity) {
        if (isAttached()) {
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(velocity.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop Velocity with torque control (requires Pro)
     *
     * @param velocity rotations per second
     */
    protected void setVelocityTCFOCrpm(DoubleSupplier velocityRPM) {
        if (isAttached()) {
            VelocityTorqueCurrentFOC output =
                    config.velocityTorqueCurrentFOC.withVelocity(
                            Conversions.RPMtoRPS(velocityRPM.getAsDouble()));
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop velocity control with voltage compensation
     *
     * @param velocity rotations per second
     */
    protected void setVelocity(DoubleSupplier velocity) {
        if (isAttached()) {
            VelocityVoltage output = config.velocityControl.withVelocity(velocity.getAsDouble());
            motor.setControl(output);
        }
    }

    /**
     * Closed-loop Position Motion Magic with torque control (requires Pro)
     *
     * @param position rotations
     */
    protected void setMMPositionFOC(DoubleSupplier position) {
        if (isAttached()) {
            MotionMagicTorqueCurrentFOC mm =
                    config.mmPositionFOC.withPosition(position.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Closed-loop Position Motion Magic
     *
     * @param position rotations
     */
    protected void setMMPosition(DoubleSupplier position) {
        setMMPosition(position, 0);
    }

    /**
     * Closed-loop Position Motion Magic using a slot other than 0
     *
     * @param position rotations
     * @param slot gains slot
     */
    public void setMMPosition(DoubleSupplier position, int slot) {
        if (isAttached()) {
            MotionMagicVoltage mm =
                    config.mmPositionVoltageSlot
                            .withSlot(slot)
                            .withPosition(position.getAsDouble());
            motor.setControl(mm);
        }
    }

    /**
     * Open-loop Percent output control with voltage compensation
     *
     * @param percent fractional units between -1 and +1
     */
    public void setPercentOutput(DoubleSupplier percent) {
        if (isAttached()) {
            VoltageOut output =
                    config.voltageControl.withOutput(
                            config.voltageCompSaturation * percent.getAsDouble());
            motor.setControl(output);
        }
    }

    public void setBrakeMode(boolean isInBrake) {
        if (isAttached()) {
            config.configNeutralBrakeMode(isInBrake);
            config.applyTalonConfig(motor);
        }
    }

    public void toggleReverseSoftLimit(boolean enabled) {
        if (isAttached()) {
            double threshold = config.talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
            config.configReverseSoftLimit(threshold, enabled);
            config.applyTalonConfig(motor);
        }
    }

    public void toggleTorqueCurrentLimit(DoubleSupplier enabledLimit, boolean enabled) {
        if (isAttached()) {
            if (enabled) {
                config.configForwardTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.configReverseTorqueCurrentLimit(enabledLimit.getAsDouble());
                config.applyTalonConfig(motor);
            } else {
                config.configForwardTorqueCurrentLimit(400);
                config.configReverseTorqueCurrentLimit(400);
                config.applyTalonConfig(motor);
            }
        }
    }

    public static class FollowerConfig {
        @Getter private String name;
        @Getter private CanDeviceId id;
        @Getter private boolean attached = true;
        @Getter private boolean opposeLeader = false;

        public FollowerConfig(String name, int id, String canbus, boolean opposeLeader) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            this.opposeLeader = opposeLeader;
        }
    }

    public static class Config {
        @Getter private String name;
        @Getter @Setter private boolean attached = true;
        @Getter private CanDeviceId id;
        @Getter protected TalonFXConfiguration talonConfig;
        @Getter private int numMotors = 1;
        @Getter private double voltageCompSaturation = 12.0; // 12V by default
        @Getter private double minRotation;
        @Getter private double maxRotation;

        @Getter private FollowerConfig[] followerConfigs = new FollowerConfig[0];

        @Getter
        private MotionMagicVelocityTorqueCurrentFOC mmVelocityFOC =
                new MotionMagicVelocityTorqueCurrentFOC(0);

        @Getter
        private MotionMagicTorqueCurrentFOC mmPositionFOC = new MotionMagicTorqueCurrentFOC(0);

        @Getter
        private MotionMagicVelocityVoltage mmVelocityVoltage = new MotionMagicVelocityVoltage(0);

        @Getter private MotionMagicVoltage mmPositionVoltage = new MotionMagicVoltage(0);

        @Getter
        private MotionMagicVoltage mmPositionVoltageSlot = new MotionMagicVoltage(0).withSlot(1);

        @Getter private VoltageOut voltageControl = new VoltageOut(0);
        @Getter private VelocityVoltage velocityControl = new VelocityVoltage(0);

        @Getter
        private VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

        @Getter
        private DutyCycleOut percentOutput =
                new DutyCycleOut(
                        0); // Percent Output control using percentage of supply voltage //Should
        // normally use VoltageOut

        public Config(String name, int id, String canbus) {
            this.name = name;
            this.id = new CanDeviceId(id, canbus);
            talonConfig = new TalonFXConfiguration();

            /* Put default config settings for all mechanisms here */
            talonConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
            talonConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
        }

        public void applyTalonConfig(TalonFX talon) {
            StatusCode result = talon.getConfigurator().apply(talonConfig);
            if (!result.isOK()) {
                DriverStation.reportWarning(
                        "Could not apply config changes to " + name + "\'s motor ", false);
            }
        }

        public void setFollowerConfigs(FollowerConfig... followers) {
            followerConfigs = followers;
        }

        public void configVoltageCompensation(double voltageCompSaturation) {
            this.voltageCompSaturation = voltageCompSaturation;
        }

        public void configCounterClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public void configClockwise_Positive() {
            talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        public void configSupplyCurrentLimit(double supplyLimit, boolean enabled) {
            talonConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = enabled;
        }

        public void configStatorCurrentLimit(double statorLimit, boolean enabled) {
            talonConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = enabled;
        }

        public void configForwardTorqueCurrentLimit(double currentLimit) {
            talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        }

        public void configReverseTorqueCurrentLimit(double currentLimit) {
            talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        }

        public void configNeutralDeadband(double deadband) {
            talonConfig.MotorOutput.DutyCycleNeutralDeadband = deadband;
        }

        public void configPeakOutput(double forward, double reverse) {
            talonConfig.MotorOutput.PeakForwardDutyCycle = forward;
            talonConfig.MotorOutput.PeakReverseDutyCycle = reverse;
        }

        public void configForwardSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
        }

        public void configReverseSoftLimit(double threshold, boolean enabled) {
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = threshold;
            talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
        }

        // Configure optional motion magic velocity parameters
        public void configMotionMagicVelocity(double acceleration, double feedforward) {
            mmVelocityFOC =
                    mmVelocityFOC.withAcceleration(acceleration).withFeedForward(feedforward);
            mmVelocityVoltage =
                    mmVelocityVoltage.withAcceleration(acceleration).withFeedForward(feedforward);
        }

        // Configure optional motion magic position parameters
        public void configMotionMagicPosition(double feedforward) {
            mmPositionFOC = mmPositionFOC.withFeedForward(feedforward);
            mmPositionVoltage = mmPositionVoltage.withFeedForward(feedforward);
        }

        public void configMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
            talonConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
            talonConfig.MotionMagic.MotionMagicAcceleration = acceleration;
            talonConfig.MotionMagic.MotionMagicJerk = jerk;
        }

        // This is the ratio of rotor rotations to the mechanism's output.
        // If a remote sensor is used this a ratio of sensor rotations to the mechanism's output.
        public void configGearRatio(double gearRatio) {
            talonConfig.Feedback.SensorToMechanismRatio = gearRatio;
        }

        public double getGearRatio() {
            return talonConfig.Feedback.SensorToMechanismRatio;
        }

        public void configNeutralBrakeMode(boolean isInBrake) {
            if (isInBrake) {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            } else {
                talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            }
        }

        /**
         * Defaults to slot 0
         *
         * @param kP
         * @param kI
         * @param kD
         */
        public void configPIDGains(double kP, double kI, double kD) {
            configPIDGains(0, kP, kI, kD);
        }

        public void configPIDGains(int slot, double kP, double kI, double kD) {
            talonConfigFeedbackPID(slot, kP, kI, kD);
        }

        /**
         * Defaults to slot 0
         *
         * @param kS
         * @param kV
         * @param kA
         * @param kG
         */
        public void configFeedForwardGains(double kS, double kV, double kA, double kG) {
            configFeedForwardGains(0, kS, kV, kA, kG);
        }

        public void configFeedForwardGains(int slot, double kS, double kV, double kA, double kG) {
            talonConfigFeedForward(slot, kV, kA, kS, kG);
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source) {
            configFeedbackSensorSource(source, 0);
        }

        public void configFeedbackSensorSource(FeedbackSensorSourceValue source, double offset) {
            talonConfig.Feedback.FeedbackSensorSource = source;
            talonConfig.Feedback.FeedbackRotorOffset = offset;
        }

        /**
         * Defaults to slot 0
         *
         * @param isArm
         */
        public void configGravityType(boolean isArm) {
            configGravityType(0, isArm);
        }

        public void configGravityType(int slot, boolean isArm) {
            GravityTypeValue gravityType =
                    isArm ? GravityTypeValue.Arm_Cosine : GravityTypeValue.Elevator_Static;
            if (slot == 0) {
                talonConfig.Slot0.GravityType = gravityType;
            } else if (slot == 1) {
                talonConfig.Slot1.GravityType = gravityType;
            } else if (slot == 2) {
                talonConfig.Slot2.GravityType = gravityType;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid slot", false);
            }
        }

        // Configure the TalonFXConfiguration feed forward gains
        private void talonConfigFeedForward(int slot, double kV, double kA, double kS, double kG) {
            if (slot == 0) {
                talonConfig.Slot0.kV = kV;
                talonConfig.Slot0.kA = kA;
                talonConfig.Slot0.kS = kS;
                talonConfig.Slot0.kG = kG;
            } else if (slot == 1) {
                talonConfig.Slot1.kV = kV;
                talonConfig.Slot1.kA = kA;
                talonConfig.Slot1.kS = kS;
                talonConfig.Slot1.kG = kG;
            } else if (slot == 2) {
                talonConfig.Slot2.kV = kV;
                talonConfig.Slot2.kA = kA;
                talonConfig.Slot2.kS = kS;
                talonConfig.Slot2.kG = kG;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid FeedForward slot", false);
            }
        }

        private void talonConfigFeedbackPID(int slot, double kP, double kI, double kD) {
            if (slot == 0) {
                talonConfig.Slot0.kP = kP;
                talonConfig.Slot0.kI = kI;
                talonConfig.Slot0.kD = kD;
            } else if (slot == 1) {
                talonConfig.Slot1.kP = kP;
                talonConfig.Slot1.kI = kI;
                talonConfig.Slot1.kD = kD;
            } else if (slot == 2) {
                talonConfig.Slot2.kP = kP;
                talonConfig.Slot2.kI = kI;
                talonConfig.Slot2.kD = kD;
            } else {
                DriverStation.reportWarning("MechConfig: Invalid Feedback slot", false);
            }
        }

        /**
         * Sets the minimum and maximum motor rotations
         *
         * @param minRotation
         * @param maxRotation
         */
        protected void configMinMaxRotations(double minRotation, double maxRotation) {
            this.minRotation = minRotation;
            this.maxRotation = maxRotation;
        }
    }
}
