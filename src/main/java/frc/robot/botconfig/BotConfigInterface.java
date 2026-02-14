package frc.robot.botconfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Interface for robot-specific tuner configuration.
 * Each implementation maps to a Tuner X-generated TunerConstants class
 * for a specific robot.
 */
public interface BotConfigInterface {

    /**
     * The theoretical free speed at 12 V applied output.
     * @see TunerConstants.kSpeedAt12Volts
     */
    LinearVelocity getSpeedAt12Volts();

    /**
     * The teleop max speed (may be divided down from kSpeedAt12Volts for testing).
     * @see TunerConstants.kSpeedInTeleop
     */
    LinearVelocity getSpeedInTeleop();

    /**
     * The margin of error (in degrees) for AprilTag alignment.
     * @see TunerConstants.kAlignmentErrorMargin
     */
    double getAlignmentErrorMargin();

    /**
     * Sensitivity threshold for manual move interruption of swerve requests.
     * @see TunerConstants.kSwerveMoveInterruptionSensitivity
     */
    double getSwerveMoveInterruptionSensitivity();

    /**
     * Sensitivity threshold for manual turn interruption of swerve requests.
     * @see TunerConstants.kSwerveTurnInterruptionSensitivity
     */
    double getSwerveTurnInterruptionSensitivity();

    /**
     * Drivetrain-wide constants (CAN bus, Pigeon ID, Pigeon configs).
     * @see TunerConstants.DrivetrainConstants
     */
    SwerveDrivetrainConstants getDrivetrainConstants();

    /**
     * Front-left swerve module constants.
     * @see TunerConstants.FrontLeft
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft();

    /**
     * Front-right swerve module constants.
     * @see TunerConstants.FrontRight
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight();

    /**
     * Back-left swerve module constants.
     * @see TunerConstants.BackLeft
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft();

    /**
     * Back-right swerve module constants.
     * @see TunerConstants.BackRight
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight();

    /**
     * Creates a CommandSwerveDrivetrain instance for this robot configuration.
     * Should only be called once in the robot program.
     * @see TunerConstants.createDrivetrain()
     */
    CommandSwerveDrivetrain createDrivetrain();
}
