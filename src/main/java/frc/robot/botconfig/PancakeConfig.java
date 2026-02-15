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
        return GeneratedPancakeConstants.createDrivetrain();
    }


    /*************************************************************************************
     *
     * INTAKE CONSTANTS
     *
     ************************************************************************************/

    @Override
    public int getIntakeMotorId() {
        return 30;
    }

    @Override
    public int getIntakeStallLimit() {
        return 40;
    }


    /*************************************************************************************
     *
     * SHOOTERCONSTANTS
     *
     ************************************************************************************/

    @Override
    public int getShooterLeftMotorId() {
        return 40;
    }

    @Override
    public int getShooterRightMotorId() {
        return 41;
    }

    @Override
    public int getShooterStatorCurrentLimit() {
        return 40;
    }

    @Override
    public int getShooterSupplyCurrentLimit() {
        return 50;
    }

    @Override
    public int getShooterHoodPwmChannel() {
        return 0;
    }


    /*************************************************************************************
     *
     * VISION
     *
     ************************************************************************************/

    @Override
    public String getVisionLimelightNameReal() {
        return "limelight-fixed";
    }
}
