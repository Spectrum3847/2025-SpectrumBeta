package frc.spectrumLib;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Telemetry extends DogLog implements Subsystem {

    public enum Fault {
        CAMERA_OFFLINE,
        AUTO_SHOT_TIMEOUT_TRIGGERED,
        BROWNOUT,
    }

    /**
     * Priority levels for printing to the console NORMAL: Low priority, only print if enabled HIGH:
     * High priority, always print
     */
    public enum PrintPriority {
        NORMAL,
        HIGH
    }

    private static PrintPriority priority = PrintPriority.HIGH;

    public Telemetry() {
        super();
        register();
    }

    public static void start(boolean ntPublish, boolean captureNt, PrintPriority priority) {
        setPriority(priority);
        Telemetry.setOptions(
                new DogLogOptions()
                        .withNtPublish(ntPublish)
                        .withCaptureDs(true)
                        .withCaptureNt(captureNt)
                        .withCaptureConsole(true)
                        .withLogExtras(true));
        Telemetry.setPdh(new PowerDistribution());
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private static void setPriority(PrintPriority priority) {
        Telemetry.priority = priority;
    }

    public static Command log(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(
                        () -> log("Commands", "Init: " + cmd.getName()),
                        () -> log("Commands", "End: " + cmd.getName())));
    }

    /** Print a statement if they are enabled */
    public static void print(String output, PrintPriority priority) {
        String out = "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output;
        if (priority == PrintPriority.HIGH || Telemetry.priority == PrintPriority.NORMAL) {
            System.out.println(out);
        }
        log("Prints", out);
    }

    public static void print(String output) {
        print(output, PrintPriority.NORMAL);
    }
}
