package robotutils;

import robotutils.drivesmooth.DriveSmooth;
import robotutils.interfaces.DriveSmoothInterface;


/** Factory for robot utility objects. */
public class RobotUtilsFactory {

    /** Creates a default drive smoothing pipeline. */
    public DriveSmoothInterface createDriveSmooth() {
        return new DriveSmooth();
    }

    /**
     * Creates a drive smoothing pipeline with explicit configuration.
     *
     * @param translationSlewRate max translation change per second
     * @param rotationSlewRate max rotation change per second
     * @param joystickDeadband deadband threshold in [0, 1)
     * @param translationExponent response exponent for translation
     * @param rotationExponent response exponent for rotation
     * @return configured drive smoothing pipeline
     */
    public DriveSmoothInterface createDriveSmooth(
        double translationSlewRate,
        double rotationSlewRate,
        double joystickDeadband,
        double translationExponent,
        double rotationExponent) {

        return new DriveSmooth(
            translationSlewRate,
            rotationSlewRate,
            joystickDeadband,
            translationExponent,
            rotationExponent);
    }

    /** Create using default params. */
    public DriveSmoothInterface createDefaultDriveSmooth() {
        return createDriveSmooth();
    }
}
