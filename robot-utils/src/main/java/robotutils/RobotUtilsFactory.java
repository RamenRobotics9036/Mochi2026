package robotutils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import robotutils.drivesmooth.DriveSmooth;
import robotutils.interfaces.DriveSmoothInterface;
import robotutils.interfaces.JoystickInputInterface;
import robotutils.joystickinput.JoystickInput;


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

    /**
     * Creates a joystick input processor.
     *
     * @param driveSmooth smoothing pipeline (deadband, curve, slew)
     * @param rawxSupplier supplier for forward/back axis
     * @param rawySupplier supplier for strafe axis
     * @param rawRotateSupplier supplier for rotation axis
     * @param finePositioningEnabledSupplier returns {@code true} when fine-positioning is enabled
     * @param teleoperatedSpeed maximum linear velocity in m/s
     * @param maxAngularRate maximum angular velocity in rad/s
     * @param isSimulation {@code true} when running in simulation
     * @param operatorForwardDegreesSupplier supplier for operator forward direction in degrees
     * @return configured joystick input processor
     */
    public JoystickInputInterface createJoystickInput(
        DriveSmoothInterface driveSmooth,
        DoubleSupplier rawxSupplier,
        DoubleSupplier rawySupplier,
        DoubleSupplier rawRotateSupplier,
        BooleanSupplier finePositioningEnabledSupplier,
        double teleoperatedSpeed,
        double maxAngularRate,
        boolean isSimulation,
        DoubleSupplier operatorForwardDegreesSupplier) {

        return new JoystickInput(
            driveSmooth,
            rawxSupplier,
            rawySupplier,
            rawRotateSupplier,
            finePositioningEnabledSupplier,
            teleoperatedSpeed,
            maxAngularRate,
            isSimulation,
            operatorForwardDegreesSupplier);
    }
}
