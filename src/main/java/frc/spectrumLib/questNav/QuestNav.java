package frc.spectrumLib.questNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import lombok.Getter;
import lombok.Setter;

public class QuestNav {
    // Configure Network Tables topics (oculus/...) to communicate with the Quest HMD
    static NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    static NetworkTable nt4Table = nt4Instance.getTable("oculus");
    private static IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private static IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Subscribe to the Network Tables oculus data topics
    @Getter
    private static IntegerSubscriber questFrameCount =
            nt4Table.getIntegerTopic("frameCount").subscribe(0);

    @Getter
    private static DoubleSubscriber questTimestamp =
            nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);

    @Getter
    private static FloatArraySubscriber questPosition =
            nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    @Getter
    private static FloatArraySubscriber questQuaternion =
            nt4Table.getFloatArrayTopic("quaternion")
                    .subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});

    @Getter
    private static FloatArraySubscriber questEulerAngles =
            nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    @Getter
    private static DoubleSubscriber questBattery =
            nt4Table.getDoubleTopic("batteryLevel").subscribe(0.0f);

    // Local heading helper variables
    @Getter @Setter private static float yawOffset = 0.0f;

    //     // Zero the relative robot heading
    //   public void zeroHeading() {
    //     float[] eulerAngles = questEulerAngles.get();
    //     yaw_offset = eulerAngles[1];
    //     angleSetpoint = 0.0;
    //   }

    //   // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
    //   public static void zeroPosition() {
    //     resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    //     if (questMiso.get() != 99) {
    //       questMosi.set(1);
    //     }
    //   }

    // Clean up oculus subroutine messages after processing on the headset
    public static void cleanUpMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    // Return the robot heading in degrees, between -180 and 180 degrees
    public static double getQuestNavHeading() {
        return Rotation2d.fromDegrees(getQuestNavYaw()).getDegrees();
    }

    // Get the rotation rate of the robot
    public static double getTurnRate() {
        return getQuestNavYaw();
    }

    // Get the yaw Euler angle of the headset
    public static float getQuestNavYaw() {
        float[] eulerAngles = questEulerAngles.get();
        var ret = eulerAngles[1] - yawOffset;
        ret %= 360;
        if (ret < 0) {
            ret += 360;
        }
        return ret;
    }

    public static Translation2d getQuestNavPosition() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
    }

    public static Pose2d getQuestNavPose() {
        var oculousPositionCompensated =
                getQuestNavPosition().minus(new Translation2d(0, 0.1651)); // 6.5
        return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getQuestNavYaw()));
    }
}
