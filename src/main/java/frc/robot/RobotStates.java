package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.crescendo.Field;
import frc.robot.RobotConfig.RobotType;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.SpectrumState;

public class RobotStates {
    private static final RobotConfig config = Robot.getRobotConfig();
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */
    public static final Trigger pm = new Trigger(() -> config.getRobotType() == RobotType.PM);

    public static final Trigger am = new Trigger(() -> config.getRobotType() == RobotType.AM);
    public static final Trigger fm = new Trigger(() -> config.getRobotType() == RobotType.FM);
    public static final Trigger sim = new Trigger(() -> config.getRobotType() == RobotType.SIM);

    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger intaking =
            pilot.intake_A.or(visionIntaking, autonIntake, operator.intake_A);
    public static final Trigger ejecting = pilot.eject_fA.or(operator.eject_fA);

    public static final Trigger ampZone =
            swerve.inXzoneAlliance(() -> 0, () -> Field.getHalfLengh() / 2)
                    .and(swerve.inYzone(() -> Field.getHalfWidth(), () -> Field.getFieldWidth()));

    public static final Trigger score = pilot.score_RB;

    public static final Trigger speakerZone =
            swerve.inXzoneAlliance(() -> 0, () -> (Field.getHalfLengh() - 1));
    public static final Trigger speakerPrep = pilot.launchPrep_RT.and(speakerZone);

    public static final Trigger subwooferPrep = pilot.subwooferPrep_fRT;
    public static final Trigger feedPrep = pilot.launchPrep_RT.and(speakerZone.not());

    public static final Trigger climbPrep = pilot.climbPrep_RDP;
    public static final Trigger climbRoutine =
            pilot.climbRoutine_start; // TODO: Add a check for hooks up

    public static final Trigger ampPrep = pilot.ampPrep_B; // .and(ampZone);
    public static final Trigger noteToAmp = pilot.ampPrep_B.or(operator.noteToAmp_B, climbPrep);

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");

    // Setup any binding to set states
    public static void setupStates() {
        pilot.coastOn_dB.and(sim.not()).onTrue(coastMode.setTrue());
        pilot.coastOff_dA.and(sim.not()).onTrue(coastMode.setFalse());
    }
}
