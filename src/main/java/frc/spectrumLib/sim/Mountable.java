package frc.spectrumLib.sim;

import frc.spectrumLib.sim.Mount.MountType;

public interface Mountable {

    default double getUpdatedX(
            MountType mountType,
            double initialX,
            double initialY,
            double initMountX,
            double initMountY,
            double initMountAngle,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        switch (mountType) {
            case LINEAR:
                return getXWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        initMountX + displacementX);
            case ARM:
                return getXWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        mountX);
            default:
                return initialX;
        }
    }

    default double getUpdatedY(
            MountType mountType,
            double initialX,
            double initialY,
            double initMountX,
            double initMountY,
            double initMountAngle,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        switch (mountType) {
            case LINEAR:
                return getYWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        initMountY + displacementY);
            case ARM:
                return getYWithAngle(
                        getDistance(initialX, initialY, initMountX, initMountY),
                        mountAngle
                                + getAngleOffset(
                                        initialX, initialY, initMountX, initMountY, initMountAngle),
                        mountY);
            default:
                return initialY;
        }
    }

    default double getUpdatedX(RollerConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedX(ArmConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedX(LinearConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(RollerConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(ArmConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(LinearConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getInitMountX(),
                config.getInitMountY(),
                config.getInitMountAngle(),
                mount.getMountX(),
                mount.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    /**
     * Returns the radians a mounted object should be away from a mount based on their initial
     * positions
     */
    static double getAngleOffset(
            double initialX, double initialY, double mountX, double mountY, double startingAngle) {
        double hypotenuse = getDistance(initialX, initialY, mountX, mountY);
        if (initialX >= mountX) {
            return Math.asin((initialY - mountY) / hypotenuse) - startingAngle;
        } else {
            return Math.toRadians(180)
                    - Math.asin((initialY - mountY) / hypotenuse)
                    - startingAngle;
        }
    }

    static double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    static double getXWithAngle(double radius, double angle, double displacementX) {
        return radius * Math.cos(angle) + displacementX;
    }

    static double getYWithAngle(double radius, double angle, double displacementY) {
        return radius * Math.sin(angle) + displacementY;
    }
}
