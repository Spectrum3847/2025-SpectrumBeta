package frc.robot.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotStates.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.crescendo.Field;
import frc.robot.Robot;
import frc.robot.climber.ClimberStates;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;
import java.util.function.DoubleSupplier;

public class SwerveStates {
    static Swerve swerve = Robot.getSwerve();
    static SwerveConfig config = Robot.getConfig().swerve;
    static Pilot pilot = Robot.getPilot();

    static Command pilotSteerCommand =
            pilotDrive().ignoringDisable(true).withName("SwerveCommands.pilotSteer");
    static SpectrumState steeringLock = new SpectrumState("SteeringLock");

    protected static void setupDefaultCommand() {
        if (Rio.id == Rio.PHOTON_2024) {
            // Use this to set a different command based on robotType
            // Robot.swerve.setDefaultCommand(PhotonPilotCommands.pilotDrive());
            // return;
            // swerve.setDefaultCommand(pilotSteerCommand);
        } else {
            swerve.setDefaultCommand(pilotSteerCommand);
        }
    }

    protected static void setStates() {

        pilot.steer.whileTrue(pilotSteerCommand); // Force back to manual steering when we steer

        // When driving and have never steered, it doesn't lock
        // When driving, and we stop steering it locks
        // When not driving it stops locking
        pilot.steer.and(pilot.driving).onTrue(steeringLock.setTrue());
        pilot.driving.onFalse(steeringLock.setFalse());
        steeringLock.and(pilot.steer.not()).onTrue(lockToClosest45degDrive());

        ampPrep.whileTrue(pilotAimDrive(() -> Field.flipAimAngleIfBlue(270)));

        // TODO:Should replace with method that gives us angle to the speaker
        // speakerPrep.whileTrue(pilotAimDrive(() -> 0));

        pilot.fpv_rs.whileTrue(fpvDrive());
        pilot.snapSteer.whileTrue(snapSteerDrive());

        pilot.upReorient.onTrue(reorientForward());
        pilot.leftReorient.onTrue(reorientLeft());
        pilot.downReorient.onTrue(reorientBack());
        pilot.rightReorient.onTrue(reorientRight());

        // Drive fwd while the climber is below mid climb and not home
        climbRoutine
                .and(ClimberStates.belowMidClimbPos, ClimberStates.atHomePos.not())
                .whileTrue(climbDrive());
    }

    /** Pilot Commands ************************************************************************ */
    /**
     * Drive the robot using left stick and control orientation using the right stick Only Cardinal
     * directions are allowed
     *
     * @return
     */
    protected static Command snapSteerDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::chooseCardinalDirections)
                .withName("Swerve.PilotStickSteer");
    }

    protected static Command pilotDrive() {
        return drive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotDrive");
    }

    protected static Command fpvDrive() {
        return fpvDrive(
                        pilot::getDriveFwdPositive,
                        pilot::getDriveLeftPositive,
                        pilot::getDriveCCWPositive)
                .withName("Swerve.PilotFPVDrive");
    }

    protected static Command pilotAimDrive(DoubleSupplier targetDegrees) {
        return aimDrive(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive, targetDegrees)
                .withName("Swerve.PilotAimDrive");
    }

    // TODO: Snake Drive, where the robot moves in the direction of the left stick, but the
    // orientation is controlled by the direction the left stick is pointing, so intake is always
    // pointing where the robot is moving

    protected static Command headingLockDrive() {
        return headingLock(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotHeadingLockDrive");
    }

    protected static Command lockToClosest45degDrive() {
        return lockToClosest45deg(pilot::getDriveFwdPositive, pilot::getDriveLeftPositive)
                .withName("Swerve.PilotLockTo45degDrive");
    }

    /** Turn the swerve wheels to an X to prevent the robot from moving */
    protected static Command xBrake() {
        return swerve.applyRequest(SwerveRequest.SwerveDriveBrake::new).withName("Swerve.Xbrake");
    }

    protected static Command climbDrive() {
        return fpvDrive(
                () -> (0.1 * config.getSpeedAt12Volts().baseUnitMagnitude()), () -> 0, () -> 0);
    }

    /**
     * ************************************************************************* Helper Commands
     * ************************************************************************
     */
    protected static Command resetTurnController() {
        return Commands.runOnce(() -> swerve.resetRotationController())
                .withName("ResetTurnController");
    }

    protected static Command setTargetHeading(DoubleSupplier targetHeading) {
        return Commands.runOnce(() -> config.setTargetHeading(targetHeading.getAsDouble()))
                .withName("SetTargetHeading");
    }

    private static final SwerveRequest.FieldCentric fieldCentricDrive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            config.getSpeedAt12Volts().in(MetersPerSecond) * config.getDeadband())
                    .withRotationalDeadband(config.getMaxAngularRate() * config.getDeadband())
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Uses m/s and rad/s
    private static Command drive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                () ->
                        fieldCentricDrive
                                .withVelocityX(fwdPositive.getAsDouble())
                                .withVelocityY(leftPositive.getAsDouble())
                                .withRotationalRate(ccwPositive.getAsDouble()));
    }

    private static final SwerveRequest.RobotCentric robotCentric =
            new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static Command fpvDrive(
            DoubleSupplier fwdPositive, DoubleSupplier leftPositive, DoubleSupplier ccwPositive) {
        return swerve.applyRequest(
                () ->
                        robotCentric
                                .withVelocityX(fwdPositive.getAsDouble())
                                .withVelocityY(leftPositive.getAsDouble())
                                .withRotationalRate(ccwPositive.getAsDouble()));
    }

    /**
     * Reset the turn controller and then run the drive command with a angle supplier. This can be
     * used for aiming at a goal or heading locking, etc.
     */
    protected static Command aimDrive(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier targetRadians) {
        return resetTurnController()
                .andThen(
                        drive(
                                velocityX,
                                velocityY,
                                () -> swerve.calculateRotationController(targetRadians)))
                .withName("Swerve.aimDrive");
    }

    // TODO: Potential new heading lock, that only fixes minor heading errors, and leaves large ones
    /**
     * Reset the turn controller, set the target heading to the current heading(end that command
     * immediately), and then run the drive command with the Rotation controller. The rotation
     * controller will only engage if you are driving x or y.
     */
    protected static Command headingLock(DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(
                        setTargetHeading(() -> swerve.getRotation().getRadians()),
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> config.getTargetHeading())))
                .withName("Swerve.HeadingLock");
    }

    protected static Command lockToClosest45deg(
            DoubleSupplier velocityX, DoubleSupplier velocityY) {
        return resetTurnController()
                .andThen(setTargetHeading(() -> swerve.getClosest45()))
                .deadlineFor(pilotDrive())
                .andThen(
                        drive(
                                velocityX,
                                velocityY,
                                rotateToHeadingWhenMoving(
                                        velocityX, velocityY, () -> swerve.getClosest45())))
                .withName("Swerve.LockTo45deg");
    }

    private static DoubleSupplier rotateToHeadingWhenMoving(
            DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier heading) {
        return () -> {
            if (Math.abs(velocityX.getAsDouble()) < 0.5
                    && Math.abs(velocityY.getAsDouble()) < 0.5) {
                return 0;
            } else {
                return swerve.calculateRotationController(heading::getAsDouble);
            }
        };
    }

    /**
     * *******************************************************************************************
     * Reorient Commands
     */
    protected static Command reorientForward() {
        return swerve.reorientPilotAngle(0).withName("Swerve.reorientForward");
    }

    protected static Command reorientLeft() {
        return swerve.reorientPilotAngle(90).withName("Swerve.reorientLeft");
    }

    protected static Command reorientBack() {
        return swerve.reorientPilotAngle(180).withName("Swerve.reorientBack");
    }

    protected static Command reorientRight() {
        return swerve.reorientPilotAngle(270).withName("Swerve.reorientRight");
    }

    protected static Command cardinalReorient() {
        return swerve.cardinalReorient().withName("Swerve.cardinalReorient");
    }
}
