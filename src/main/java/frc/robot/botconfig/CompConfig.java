package frc.robot.botconfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.GeneratedCompConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Adapter that bridges BotConfigInterface to the current constants.
 * For now this delegates directly to GeneratedCompConstants; when a second robot
 * is added, this class can be swapped or extended to select between configs.
 */
public class CompConfig implements BotConfigInterface {

    /*************************************************************************************
     *
     * SWERVEDRIVE GENERATED CONSTANTS
     *
     ************************************************************************************/

    @Override
    public String getConfigName() {
        return "Competition Bot Config";
    }

    @Override
    public CANBus getCANBus() {
        return GeneratedCompConstants.kCANBus;
    }

    @Override
    public LinearVelocity getSpeedAt12Volts() {
        return GeneratedCompConstants.kSpeedAt12Volts;
    }

    @Override
    public LinearVelocity getSpeedInTeleop() {
        return GeneratedCompConstants.kSpeedInTeleop;
    }

    @Override
    public double getAlignmentErrorMargin() {
        return GeneratedCompConstants.kAlignmentErrorMargin;
    }

    @Override
    public double getSwerveMoveInterruptionSensitivity() {
        return GeneratedCompConstants.kSwerveMoveInterruptionSensitivity;
    }

    @Override
    public double getSwerveTurnInterruptionSensitivity() {
        return GeneratedCompConstants.kSwerveTurnInterruptionSensitivity;
    }

    @Override
    public SwerveDrivetrainConstants getDrivetrainConstants() {
        return GeneratedCompConstants.DrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontLeft() {
        return GeneratedCompConstants.FrontLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getFrontRight() {
        return GeneratedCompConstants.FrontRight;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackLeft() {
        return GeneratedCompConstants.BackLeft;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getBackRight() {
        return GeneratedCompConstants.BackRight;
    }

    @Override
    public CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            GeneratedCompConstants.DrivetrainConstants,
            GeneratedCompConstants.FrontLeft,
            GeneratedCompConstants.FrontRight,
            GeneratedCompConstants.BackLeft,
            GeneratedCompConstants.BackRight);
    }

    /*************************************************************************************
     *
     * FORCE DISABLE SUBSYSTEMS (e.g. Pancake is missing several)
     *
     ************************************************************************************/

    /** Force disable shooter. */
    @Override
    public boolean shouldForceDisableShooter() {
        return false;
    }

    /** Force disable indexer. */
    @Override
    public boolean shouldForceDisableIndexer() {
        return false;
    }

    /** Force disable spinny. */
    @Override
    public boolean shouldForceDisableSpinny() {
        return false;
    }

    /** Force disable climber. */
    @Override
    public boolean shouldForceDisableClimber() {
        return false;
    }

    /** Force disable intake. */
    @Override
    public boolean shouldForceDisableIntake() {
        return false;
    }

    /** Force disable intake arm. */
    @Override
    public boolean shouldForceDisableIntakeArm() {
        return false;
    }
}
