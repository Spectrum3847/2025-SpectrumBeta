// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.SpectrumSubsystem;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements SpectrumSubsystem, NTSendable {
    private SwerveConfig config;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private RotationController rotationController;

    @Getter
    protected SwerveModuleState[] setpoints =
            new SwerveModuleState[] {}; // This currently doesn't do anything

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedPilotPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /**
     * Constructs a new Swerve drive subsystem.
     *
     * @param config The configuration object containing drivetrain constants and module
     *     configurations.
     */
    public Swerve(SwerveConfig config) {
        super(config.getDrivetrainConstants(), config.getModules());
        // this.robotConfig = robotConfig;
        this.config = config;
        configurePathPlanner();

        rotationController = new RotationController(config);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        SendableRegistry.add(this, "SwerveDrive");
        Robot.subsystems.add(this);
        CommandScheduler.getInstance().registerSubsystem(this);
        RobotTelemetry.print(getName() + " Subsystem Initialized: ");
    }

    /**
     * This method is called periodically and is used to update the pilot's perspective. It ensures
     * that the swerve drive system is aligned correctly based on the pilot's view.
     */
    @Override
    public void periodic() {
        setPilotPerspective();
    }

    public void setupStates() {
        SwerveStates.setStates();
    };

    public void setupDefaultCommand() {
        SwerveStates.setupDefaultCommand();
    };

    /**
     * The `initSendable` function sets up properties for a SmartDashboard type "SwerveDrive" with
     * position and velocity double values.
     *
     * @param builder The `builder` parameter is an instance of the `NTSendableBuilder` class
     */
    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Position", () -> 2, null);
        builder.addDoubleProperty("Velocity", () -> 4, null);
    }

    /**
     * The function `getRobotPose` returns the robot's pose after checking and updating it.
     *
     * @return The `getRobotPose` method is returning the robot's current pose after calling the
     *     `seedCheckedPose` method with the current pose as an argument.
     */
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        return keepPoseOnField(pose);
    }

    // Keep the robot on the field
    private Pose2d keepPoseOnField(Pose2d pose) {
        double halfRobot = config.getRobotLength() / 2;
        double x = pose.getX();
        double y = pose.getY();

        double newX = Util.limit(x, halfRobot, Field.getFieldLength() - halfRobot);
        double newY = Util.limit(y, halfRobot, Field.getFieldWidth() - halfRobot);

        if (x != newX || y != newY) {
            pose = new Pose2d(new Translation2d(newX, newY), pose.getRotation());
            resetPose(pose);
        }
        return pose;
    }

    public Trigger inXzone(DoubleSupplier minXmeter, DoubleSupplier maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                () -> getRobotPose().getX(),
                                () -> minXmeter.getAsDouble(),
                                () -> maxXmeter.getAsDouble()));
    }

    public Trigger inYzone(DoubleSupplier minYmeter, DoubleSupplier maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                () -> getRobotPose().getY(),
                                () -> minYmeter.getAsDouble(),
                                () -> maxYmeter.getAsDouble()));
    }

    /**
     * This method is used to check if the robot is in the X zone of the field flips the values if
     * Red Alliance
     *
     * @param minXmeter
     * @param maxXmeter
     * @return
     */
    public Trigger inXzoneAlliance(DoubleSupplier minXmeter, DoubleSupplier maxXmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                Field.flipXifRed(getRobotPose().getX()),
                                minXmeter.getAsDouble(),
                                maxXmeter.getAsDouble()));
    }

    /**
     * This method is used to check if the robot is in the Y zone of the field flips the values if
     * Red Alliance
     *
     * @param minYmeter
     * @param maxYmeter
     * @return
     */
    public Trigger inYzoneAlliance(DoubleSupplier minYmeter, DoubleSupplier maxYmeter) {
        return new Trigger(
                () ->
                        Util.inRange(
                                Field.flipYifRed(getRobotPose().getY()),
                                minYmeter.getAsDouble(),
                                maxYmeter.getAsDouble()));
    }

    // Used to set a control request to the swerve module, ignores disable so commands are
    // continuous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private void setPilotPerspective() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedPilotPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            (allianceColor) -> {
                                this.setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? config.getRedAlliancePerspectiveRotation()
                                                : config.getBlueAlliancePerspectiveRotation());
                                hasAppliedPilotPerspective = true;
                            });
        }
    }

    protected void reorient(double angleDegrees) {
        resetPose(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorientPilotAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output;
                    output = Field.flipTrueAngleIfRed(angleDegrees);
                    reorient(output);
                });
    }

    protected double getClosestCardinal() {
        double heading = getRotation().getRadians();
        if (heading > -Math.PI / 4 && heading <= Math.PI / 4) {
            return 0;
        } else if (heading > Math.PI / 4 && heading <= 3 * Math.PI / 4) {
            return 90;
        } else if (heading > 3 * Math.PI / 4 || heading <= -3 * Math.PI / 4) {
            return 180;
        } else {
            return 270;
        }
    }

    protected double getClosest45() {
        double angleRadians = getRotation().getRadians();
        double angleDegrees = Math.toDegrees(angleRadians);

        // Normalize the angle to be within 0 to 360 degrees
        angleDegrees = angleDegrees % 360;
        if (angleDegrees < 0) {
            angleDegrees += 360;
        }

        // Round to the nearest multiple of 45 degrees
        double closest45Degrees = Math.round(angleDegrees / 45.0) * 45.0;

        // Convert back to radians and return as a Rotation2d
        return Rotation2d.fromDegrees(closest45Degrees).getRadians();
    }

    protected Command cardinalReorient() {
        return runOnce(
                () -> {
                    double angleDegrees = getClosestCardinal();
                    reorient(angleDegrees);
                });
    }

    // --------------------------------------------------------------------------------
    // Rotation Controller
    // --------------------------------------------------------------------------------
    double getRotationControl(double goalRadians) {
        return rotationController.calculate(goalRadians, getRotationRadians());
    }

    void resetRotationController() {
        rotationController.reset(getRotationRadians());
    }

    Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    double getRotationRadians() {
        return getRobotPose().getRotation().getRadians();
    }

    double calculateRotationController(DoubleSupplier targetRadians) {
        return rotationController.calculate(targetRadians.getAsDouble(), getRotationRadians());
    }

    // --------------------------------------------------------------------------------
    // Vision
    // --------------------------------------------------------------------------------
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            double xStdDev,
            double yStdDev,
            double thetaStdDev) {
        Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xStdDev, yStdDev, thetaStdDev);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    // --------------------------------------------------------------------------------
    // Path Planner
    // --------------------------------------------------------------------------------
    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        resetPose(
                new Pose2d(
                        Units.feetToMeters(27),
                        Units.feetToMeters(27 / 2),
                        config.getBlueAlliancePerspectiveRotation()));
        double driveBaseRadius = .4;
        for (var moduleLocation : getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        ModuleConfig moduleConfig =
                new ModuleConfig(
                        config.getWheelRadius(),
                        config.getSpeedAt12Volts(),
                        1,
                        DCMotor.getKrakenX60(1),
                        config.getSlipCurrent(),
                        1);
        RobotConfig robotConfig = // Have directly call this to avoid name space problem
                new RobotConfig(
                        Units.lbsToKilograms(150),
                        1,
                        moduleConfig,
                        Units.inchesToMeters(26),
                        Units.inchesToMeters(
                                26)); // TODO Fix this line and line above with real numbers
        AutoBuilder.configure(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) ->
                        this.setControl(
                                AutoRequest.withSpeeds(
                                        speeds)), // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0), Robot.kDefaultPeriod),
                robotConfig,
                () ->
                        DriverStation.getAlliance().orElse(Alliance.Blue)
                                == Alliance.Red, // Assume the path needs to be flipped for Red vs
                // Blue, this is normally
                // the case
                this); // Subsystem for requirements
    }

    // --------------------------------------------------------------------------------
    // Simulation
    // --------------------------------------------------------------------------------
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(config.getSimLoopPeriod());
    }
}
