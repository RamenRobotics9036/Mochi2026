package robotutils;

import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import robotutils.drivesmooth.DriveSmooth;
import robotutils.interfaces.ArmIoInterface;
import robotutils.interfaces.DriveSmoothInterface;
import robotutils.interfaces.ElevatorIoInterface;
import robotutils.interfaces.JoystickInputInterface;
import robotutils.interfaces.MacKey;
import robotutils.interfaces.PerRobotConfigInterface;
import robotutils.interfaces.RollerIoInterface;
import robotutils.interfaces.TwoMotorRollerIoInterface;
import robotutils.joystickinput.JoystickInput;
import robotutils.perrobotconfig.PerRobotConfig;
import robotutils.sim.armsim.ArmIoSim;
import robotutils.sim.elevatorssim.ElevatorIoSim;
import robotutils.sim.rollerssim.RollerIoSim;
import robotutils.sim.rollerssim.TwoMotorRollerIoSim;


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

    /**
     * Creates a per-robot configuration selector.
     *
     * @param macToRobotNameDict maps MAC key suffixes to robot names
     * @param robotNameToConfigNameDict maps robot names to config names
     * @param configNameToConfigObjDict maps config names to config objects
     * @param defaultConfigName config name used when no MAC address matches
     * @param simulationConfigName config name used in simulation
     * @return configured per-robot config selector
     */
    public <T> PerRobotConfigInterface<T> createPerRobotConfig(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        return new PerRobotConfig<T>(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName);
    }

    /**
     * Creates a simulated arm IO implementation.
     *
     * @param deviceName simulation device name
     * @param minArmAngleDegrees minimum arm angle in degrees
     * @param maxArmAngleDegrees maximum arm angle in degrees
     * @param momentOfInertia arm moment of inertia
     * @param armLengthMeters arm length in meters
     * @param gearRatio arm gear ratio
     * @return configured arm IO simulation
     */
    public ArmIoInterface createArmIoSim(
        String deviceName,
        double minArmAngleDegrees,
        double maxArmAngleDegrees,
        double momentOfInertia,
        double armLengthMeters,
        double gearRatio) {

        return new ArmIoSim(
            deviceName,
            minArmAngleDegrees,
            maxArmAngleDegrees,
            momentOfInertia,
            armLengthMeters,
            gearRatio);
    }

    /**
     * Creates a simulated elevator IO implementation.
     *
     * @param deviceName simulation device name
     * @param gearRatio elevator gear ratio
     * @param carriageMassKg carriage mass in kilograms
     * @param drumRadiusMeters drum radius in meters
     * @param minHeightMeters minimum height in meters
     * @param maxHeightMeters maximum height in meters
     * @return configured elevator IO simulation
     */
    public ElevatorIoInterface createElevatorIoSim(
        String deviceName,
        double gearRatio,
        double carriageMassKg,
        double drumRadiusMeters,
        double minHeightMeters,
        double maxHeightMeters) {

        return new ElevatorIoSim(
            deviceName,
            gearRatio,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters);
    }

    /**
     * Creates a simulated single-motor roller IO implementation.
     *
     * @param deviceName simulation device name
     * @param momentOfInertia roller moment of inertia
     * @param gearRatio roller gear ratio
     * @return configured roller IO simulation
     */
    public RollerIoInterface createRollerIoSim(
        String deviceName,
        double momentOfInertia,
        double gearRatio) {

        return new RollerIoSim(
            deviceName,
            momentOfInertia,
            gearRatio);
    }

    /**
     * Creates a simulated two-motor roller IO implementation.
     *
     * @param deviceName simulation device name
     * @param momentOfInertia roller moment of inertia
     * @param gearRatio roller gear ratio
     * @return configured two-motor roller IO simulation
     */
    public TwoMotorRollerIoInterface createTwoMotorRollerIoSim(
        String deviceName,
        double momentOfInertia,
        double gearRatio) {

        return new TwoMotorRollerIoSim(
            deviceName,
            momentOfInertia,
            gearRatio);
    }
}
