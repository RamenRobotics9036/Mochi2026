package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.GeneratedPancakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Adapter that bridges BotConfigInterface to the current constants.
 * For now this delegates directly to GeneratedPancakeConstants; when a second robot
 * is added, this class can be swapped or extended to select between configs.
 */
public class PancakeConfig implements BotConfigInterface {

    /*************************************************************************************
     *
     * SWERVEDRIVE GENERATED CONSTANTS
     *
     ************************************************************************************/

    @Override
    public String getConfigName() {
        return "Pancake Bot Config";
    }

    @Override
    public CANBus getCANBus() {
        return GeneratedPancakeConstants.kCANBus;
    }

    @Override
    public LinearVelocity getSpeedAt12Volts() {
        return GeneratedPancakeConstants.kSpeedAt12Volts;
    }

    @Override
    public LinearVelocity getSpeedInTeleop() {
        return GeneratedPancakeConstants.kSpeedInTeleop;
    }

    @Override
    public double getAlignmentErrorMargin() {
        return GeneratedPancakeConstants.kAlignmentErrorMargin;
    }

    @Override
    public double getSwerveMoveInterruptionSensitivity() {
        return GeneratedPancakeConstants.kSwerveMoveInterruptionSensitivity;
    }

    @Override
    public double getSwerveTurnInterruptionSensitivity() {
        return GeneratedPancakeConstants.kSwerveTurnInterruptionSensitivity;
    }

    @Override
    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return GeneratedPancakeConstants.DrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
        return GeneratedPancakeConstants.FrontLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
        return GeneratedPancakeConstants.FrontRight;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
        return GeneratedPancakeConstants.BackLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
        return GeneratedPancakeConstants.BackRight;
    }

    @Override
    public CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            GeneratedPancakeConstants.DrivetrainConstants,
            GeneratedPancakeConstants.FrontLeft,
            GeneratedPancakeConstants.FrontRight,
            GeneratedPancakeConstants.BackLeft,
            GeneratedPancakeConstants.BackRight);
    }


    /*************************************************************************************
     *
     * FORCE DISABLE SUBSYSTEMS (e.g. Pancake is missing several)
     *
     ************************************************************************************/

    /** Force disable shooter. */
    @Override
    public boolean shouldForceDisableShooter() {
        return true;
    }

    /** Force disable indexer. */
    @Override
    public boolean shouldForceDisableIndexer() {
        return true;
    }

    /** Force disable spinny. */
    @Override
    public boolean shouldForceDisableSpinny() {
        return true;
    }

    /** Force disable climber. */
    @Override
    public boolean shouldForceDisableClimber() {
        return true;
    }

    /** Force disable intake. */
    @Override
    public boolean shouldForceDisableIntake() {
        return true;
    }

    /** Force disable intake arm. */
    @Override
    public boolean shouldForceDisableIntakeArm() {
        return true;
    }
}
