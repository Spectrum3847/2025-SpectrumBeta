// Based on
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Swerve extends SwerveDrivetrain implements Subsystem, NTSendable {
    private SwerveConfig config;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private RotationController rotationController;

    @Getter
    protected SwerveModuleState[] setpoints =
            new SwerveModuleState[] {}; // This currently doesn't do anything

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedPilotPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
            new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveConfig config) {
        super(config.getDrivetrainConstants(), config.getModules());
        this.config = config;
        configurePathPlanner();

        rotationController = new RotationController(config);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        SendableRegistry.add(this, "SwerveDrive");
        RobotTelemetry.print(getName() + " Subsystem Initialized: ");
    }

    @Override
    public void periodic() {
        setPilotPerspective();
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Position", () -> 2, null);
        builder.addDoubleProperty("Velocity", () -> 4, null);
    }

    // This allows us to keep the robot pose on the sim field
    // May need to make this only change the pose if we are in Sim
    public Pose2d getRobotPose() {
        Pose2d pose = getState().Pose;
        seedCheckedPose(pose);
        return getState().Pose;
    }

    // Keep the robot on the field during simulation
    private void seedCheckedPose(Pose2d pose) {

        double halfRobot = config.getRobotLength() / 2;
        double maxX = Field.getFieldLength() - halfRobot;
        double x = pose.getX();
        boolean update = false;
        if (x < halfRobot) {
            x = halfRobot;
            update = true;
        } else if (x > maxX) {
            x = maxX;
            update = true;
        }

        double maxY = Field.getFieldWidth() - halfRobot;
        double y = pose.getY();
        if (y < halfRobot) {
            y = halfRobot;
            update = true;
        } else if (y > maxY) {
            y = maxY;
            update = true;
        }
        if (update) {
            seedFieldRelative(new Pose2d(new Translation2d(x, y), pose.getRotation()));
        }
    }

    // Used to set a control request to the swerve module, ignores disable so commands are
    // continous.
    Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).ignoringDisable(true);
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void writeSetpoints(SwerveModuleState[] setpoints) {
        try {
            m_stateLock.lock();

            this.setpoints = setpoints;
        } finally {
            m_stateLock.unlock();
        }
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
        seedFieldRelative(
                new Pose2d(
                        getRobotPose().getX(),
                        getRobotPose().getY(),
                        Rotation2d.fromDegrees(angleDegrees)));
    }

    protected Command reorientPilotAngle(double angleDegrees) {
        return runOnce(
                () -> {
                    double output;
                    if (Field.isRed()) {
                        output = (angleDegrees + 180) % 360;
                    } else {
                        output = angleDegrees;
                    }
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
    // Path Planner
    // --------------------------------------------------------------------------------
    private void configurePathPlanner() {
        // Seed robot to mid field at start (Paths will change this starting position)
        seedFieldRelative(
                new Pose2d(
                        Units.feetToMeters(27),
                        Units.feetToMeters(27 / 2),
                        config.getBlueAlliancePerspectiveRotation()));
        double driveBaseRadius = .4;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        ModuleConfig moduleConfig =
                new ModuleConfig(
                        Units.inchesToMeters(config.getWheelRadiusInches()),
                        config.getSpeedAt12VoltsMps(),
                        1,
                        DCMotor.getKrakenX60(1),
                        config.getSlipCurrentA(),
                        1);
        RobotConfig robotConfig =
                new RobotConfig(
                        Units.lbsToKilograms(150),
                        1,
                        moduleConfig,
                        Units.inchesToMeters(26),
                        Units.inchesToMeters(
                                26)); // TODO Fix this line and line above with real numbers
        AutoBuilder.configure(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
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

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
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
