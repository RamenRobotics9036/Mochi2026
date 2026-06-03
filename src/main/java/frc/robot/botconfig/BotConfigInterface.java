package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.GeneratedCompConstants;
import frc.robot.generated.GeneratedPancakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

/**
 * Interface for robot-specific tuner configuration.
 * Each implementation maps to a Tuner X-generated class
 * for a specific robot.
 */
public interface BotConfigInterface {

    /*************************************************************************************
     *
     * SWERVEDRIVE GENERATED CONSTANTS
     *
     ************************************************************************************/

    /** Config name (e.g. Competition Bot, Pancake Bot). */
    String getConfigName();

    /**
     * The CAN bus used by this robot's devices.
     * @see GeneratedCompConstants.kCANBus
     * @see GeneratedPancakeConstants.kCANBus
     */
    CANBus getCANBus();

    /**
     * The theoretical free speed at 12 V applied output.
     * @see GeneratedPancakeConstants.kSpeedAt12Volts
     */
    LinearVelocity getSpeedAt12Volts();

    /**
     * The teleop max speed (may be divided down from kSpeedAt12Volts for testing).
     * @see GeneratedPancakeConstants.kSpeedInTeleop
     */
    LinearVelocity getSpeedInTeleop();

    /**
     * The margin of error (in degrees) for AprilTag alignment.
     * @see GeneratedPancakeConstants.kAlignmentErrorMargin
     */
    double getAlignmentErrorMargin();

    /**
     * Sensitivity threshold for manual move interruption of swerve requests.
     * @see GeneratedPancakeConstants.kSwerveMoveInterruptionSensitivity
     */
    double getSwerveMoveInterruptionSensitivity();

    /**
     * Sensitivity threshold for manual turn interruption of swerve requests.
     * @see GeneratedPancakeConstants.kSwerveTurnInterruptionSensitivity
     */
    double getSwerveTurnInterruptionSensitivity();

    /**
     * Drivetrain-wide constants (CAN bus, Pigeon ID, Pigeon configs).
     * @see GeneratedPancakeConstants.DrivetrainConstants
     */
    SwerveDrivetrainConstants getDrivetrainConstants();

    /**
     * Front-left swerve module constants.
     * @see GeneratedPancakeConstants.FrontLeft
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft();

    /**
     * Front-right swerve module constants.
     * @see GeneratedPancakeConstants.FrontRight
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight();

    /**
     * Back-left swerve module constants.
     * @see GeneratedPancakeConstants.BackLeft
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft();

    /**
     * Back-right swerve module constants.
     * @see GeneratedPancakeConstants.BackRight
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight();

    /**
     * Creates a CommandSwerveDrivetrain instance for this robot configuration.
     * Should only be called once in the robot program.
     */
    CommandSwerveDrivetrain createDrivetrain();


    /*************************************************************************************
     *
     * FORCE DISABLE SUBSYSTEMS (e.g. Pancake is missing several)
     *
     ************************************************************************************/

    /** Force disable shooter. */
    boolean shouldForceDisableShooter();

    /** Force disable indexer. */
    boolean shouldForceDisableIndexer();

    /** Force disable spinny. */
    boolean shouldForceDisableSpinny();

    /** Force disable climber. */
    boolean shouldForceDisableClimber();

    /** Force disable intake. */
    boolean shouldForceDisableIntake();

    /** Force disable intake arm. */
    boolean shouldForceDisableIntakeArm();


    /*************************************************************************************
     *
     * VISION CAMERAS
     *
     ************************************************************************************/

    /** Returns the list of Limelight cameras on this robot. */
    List<CameraConfig> getCameras();
}
