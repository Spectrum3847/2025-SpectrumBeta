package frc.spectrumLib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import lombok.Getter;
import lombok.Setter;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public abstract class Camera {

    public static class Config {
        /** Must match to the name given in LL dashboard */
        @Getter private final String name;

        @Getter @Setter private boolean attached = true;
        @Getter @Setter private boolean isPhotonCamera = false;
        double avgErrorPx = 0.25;
        double errorStdDevPx = 0.08;
        double fps = 20;
        double avgLatencyMs = 35.0;
        double latencyStdDevMs = 5.0;
        int resWidth = 960;
        int resHeight = 720;
        Rotation2d fovDiag = Rotation2d.fromDegrees(90);

        @Getter @Setter private boolean isIntegrating;
        /** Physical Config */
        @Getter private double forward, right, up; // meters

        @Getter private double roll, pitch, yaw; // degrees

        Translation3d robotToCameraTranslation; // Location of the camera on the robot
        Rotation3d robotToCameraRotation; // Rotation of the camera on the robot

        public Config(String name) {
            this.name = name;
        }

        /**
         * @param forward (meters) forward from center of robot
         * @param right (meters) right from center of robot
         * @param up (meters) up from center of robot
         * @return
         */
        public Config withTranslation(double forward, double right, double up) {
            this.forward = forward;
            this.right = right;
            this.up = up;
            robotToCameraTranslation = new Translation3d(forward, right, up);
            return this;
        }

        /**
         * @param roll (degrees) roll of limelight || positive is rotated right
         * @param pitch (degrees) pitch of limelight || positive is camera tilted up
         * @param yaw (yaw) yaw of limelight || positive is rotated left
         * @return
         */
        public Config withRotation(double roll, double pitch, double yaw) {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
            robotToCameraRotation = new Rotation3d(roll, pitch, yaw);
            return this;
        }

        public Config withResolution(int width, int height) {
            resWidth = width;
            resHeight = height;
            return this;
        }

        public Config withFovDiag(double fovDiag) {
            this.fovDiag = Rotation2d.fromDegrees(fovDiag);
            return this;
        }

        public Config withCalibration(double avgErrorPx, double errorStdDevPx) {
            this.avgErrorPx = avgErrorPx;
            this.errorStdDevPx = errorStdDevPx;
            return this;
        }

        public Config withFPS(double fps) {
            this.fps = fps;
            return this;
        }

        public Config withLatency(double avgLatencyMs, double latencyStdDevMs) {
            this.avgLatencyMs = avgLatencyMs;
            this.latencyStdDevMs = latencyStdDevMs;
            return this;
        }

        public SimCameraProperties getSimCameraProperties() {
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibError(avgErrorPx, errorStdDevPx);
            props.setFPS(fps);
            props.setAvgLatencyMs(avgLatencyMs);
            props.setLatencyStdDevMs(latencyStdDevMs);
            return props;
        }
    }

    @Getter private PhotonCamera photonCamera;
    PhotonCameraSim cameraSim;
    private Config config;

    public Camera(Config config) {
        this.config = config;
        if (config.isAttached() && (Robot.isSimulation() || config.isPhotonCamera())) {
            photonCamera = new PhotonCamera(config.getName());

            cameraSim = new PhotonCameraSim(photonCamera, config.getSimCameraProperties());
        }
    }

    public Camera addToVisionSystem(VisionSystemSim visionSim) {
        if (cameraSim != null) {
            Transform3d robotToCamera =
                    new Transform3d(config.robotToCameraTranslation, config.robotToCameraRotation);
            visionSim.addCamera(cameraSim, robotToCamera);
        }
        return this;
    }
}
